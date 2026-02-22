#!/usr/bin/env python3
import socket
import time
import threading
import random


# -------------------------
# Default settings
# -------------------------
DEFAULT_HOST = "192.168.100.1"
DEFAULT_PORT = 5002
TIMEOUT = 5                  # Socket timeout in seconds
DELAY_BETWEEN_CMDS = 0.2     # seconds to wait after each command
POSITION_CHECK_INTERVAL = 0.1  # seconds between position checks
MOVEMENT_THRESHOLD = 0.0001   # minimum change to consider as movement
ENCODER_0_READING = -517830855
ENCODER_PPU = 26214.4
STATIONARY_THRESHOLD = 5  # Number of consecutive stationary readings
MIN_POSITION_MM = 100.0  # Minimum valid position in mm
MAX_POSITION_MM = 2000.0  # Maximum valid position in 
MAX_VEL = 1000.0

# -------------------------
# Socket-based client class
# -------------------------
class SocketTelnetClient:
    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=TIMEOUT):
        """Establish a persistent socket connection."""
        self.host = host
        self.port = port
        self.timeout = timeout

        # Initialize monitoring attributes BEFORE creating sockets
        self.is_moving = False
        self._monitor_thread = None
        self._monitor_running = False
        self._last_position = None

        self.main_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.main_sock.settimeout(self.timeout)
        self.main_sock.connect((self.host, self.port))

        self._monitor_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._monitor_sock.settimeout(self.timeout)
        self._monitor_sock.connect((self.host, self.port))
        
        self.zero_pose = ENCODER_0_READING / ENCODER_PPU
        print(f"Zero pose set to {self.zero_pose} user units.")
    def start_monitoring(self):
        """Start the position monitoring thread."""
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._monitor_running = True
            self._monitor_thread = threading.Thread(target=self._monitor_position, daemon=True)
            self._monitor_thread.start()

    def stop_monitoring(self):
        """Stop the position monitoring thread."""
        self._monitor_running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        self._monitor_sock.close()

    def send_telnet(self, sock: socket.socket, message: str, blocking=True) -> list[str]:
        """
        Send a string message over the specified socket and read the response.
        Ensures the message ends with CRLF before sending.
        Returns the response as a list of strings (lines).
        """
        if not message.endswith("\r\n"):
            message = message + "\r\n"
        sock.sendall(message.encode("ascii"))
        if not blocking:
            return []

        # Read response until finished
        response = ""
        try:
            finished_reading = False
            while not finished_reading:
                data = sock.recv(4096)
                if not data:
                    finished_reading = True
                else:
                    decoded_data = data.decode("ascii", errors="replace")
                    response += decoded_data
                    # Check if we received a prompt indicating completion
                    if decoded_data.endswith(">"):
                        finished_reading = True
        except socket.timeout:
            print(f"Socket read timeout reached {message=}")

        response_list = response.strip().split('\r\n')
        if response_list and response_list[-1].endswith('>'):
            response_list = response_list[:-1]
        return response_list
    
    def init_motor(self):
        prog0_response = self.send_telnet(self.main_sock, "PROG0")
        # print(f"[Init motor]{prog0_response}")
        drive_response = self.send_telnet(self.main_sock, "DRIVE ON X")

        # acc_response = self.send_telnet(self.main_sock, "JOG ACC X 500")
        # dec_response = self.send_telnet(self.main_sock, "JOG DEC X 500")


        # print(f"[Init motor]{drive_response}")

    def set_velocity(self, velocity_mm_s: float):
        dir_cmd = None
        if velocity_mm_s == 0:
            dir_cmd = "JOG OFF X"
        elif velocity_mm_s < 0:
            dir_cmd = "JOG REV X"
        elif velocity_mm_s > 0:
            dir_cmd = "JOG FWD X"
        
    def goto_pose(self, user_units) -> list[str]:
        user_units = float(user_units)
        user_units = min(MAX_POSITION_MM, max(user_units, MIN_POSITION_MM))
        target_user_units = self.zero_pose - user_units
        # print(f"Going to user units: f({float(user_units)}, {self.zero_pose})={float(target_user_units):.4f}")

        cmd = f"MOV X {target_user_units}"
        print(f"[Goto pose] Sending command: {cmd}")
        response = None
        while response is None or len(response) > 1:
            response = self.send_telnet(self.main_sock, cmd)
            print(f"[Goto pose]{response}")
            if len(response) > 1:
                self.init_motor()
                print(f"[Goto pose] Re-sending command after re-init: {cmd}")
        return response

    def get_position(self, socket) -> float:
        """Get current position in user units."""
        response = self.send_telnet(socket, "PRINT(P12290/P12375)")
        # print(f"[Get position] Response: {response}")
        if response and len(response) > 0:
            try:
                user_units = float(response[1])
                location = -1*(user_units - self.zero_pose)
                return location
            except (ValueError, IndexError):
                return float("nan")
        return float("nan")

    def _monitor_position(self):
        """Background thread that monitors position changes."""
        last_position = float("-inf")
        stationary_count = 0

        while self._monitor_running:
            try:
                # Send position query using send_telnet
                # Parse position from response
                current_position = self.get_position(self._monitor_sock)
                self._last_position = current_position

                # Update is_moving flag
                if last_position is not None:
                    position_delta = abs(current_position - last_position)
                    if position_delta > MOVEMENT_THRESHOLD:
                        self.is_moving = True
                        stationary_count = 0
                    else:
                        stationary_count += 1
                        if stationary_count >= STATIONARY_THRESHOLD:
                            self.is_moving = False

                last_position = current_position

                print(f"[Monitor] Pos: {current_position:.4f}, Moving: {self.is_moving}")

                time.sleep(POSITION_CHECK_INTERVAL)

            except socket.timeout:
                print(f"[Monitor thread] Socket read timeout")
                time.sleep(POSITION_CHECK_INTERVAL)
            except Exception as e:
                print(f"[Monitor thread] Error: {e}")
                time.sleep(POSITION_CHECK_INTERVAL)

        self._monitor_sock.close()

    def close(self):
        """Close the socket connection and stop monitoring."""
        self.stop_monitoring()
        try:
            self.main_sock.close()
        except:
            pass

    def set_velocity(self, velocity_m_s: float):
        velocity_mm_s = abs(velocity_m_s) * 1000.0
        cmd = f"VEL {velocity_mm_s}"
        self.send_telnet(self.main_sock, cmd, blocking=False)

    def measure_throughput(self, n_seconds: float) -> dict:
        """
        Measure message throughput by sending random commands and waiting for responses.

        Args:
            n_seconds: Duration to run the test in seconds

        Returns:
            Dictionary with throughput statistics
        """
        # Define the message types to send
        i = 10
        def get_random_message():
            nonlocal i
            #msg_type = random.choice(['goto_pose', 'init_motor', 'monitoring'])
            # msg_type = random.choice(['goto_pose'])
            msg_type = random.choice(['velocity'])

            if msg_type == 'goto_pose':
                position_mm = min(i*10, 2000)
                i+=1
                if position_mm == 2000:
                    return "done", 'done'
                position_mm = min(MAX_POSITION_MM, max(position_mm, MIN_POSITION_MM))
                target = self.zero_pose - position_mm
                return f"MOV X {target}", 'goto_pose'
            elif msg_type == 'init_motor':
                cmd = random.choice(["PROG0", "DRIVE ON X"])
                return cmd, 'init_motor'
            elif msg_type == 'velocity':
                velocity = random.uniform(-MAX_VEL, MAX_VEL)
                cmd = f"VEL {velocity}"
                return cmd, 'velocity'
            else:  # monitoring
                cmd = random.choice([
                    "PRINT(P12290/P12375)",  # position
                    "PRINT(P28741*60)",       # velocity
                ])
                return cmd, 'monitoring'

        start_time = time.time()
        end_time = start_time + n_seconds

        total_count = 0
        success_count = 0
        error_count = 0
        counts_by_type = {'goto_pose': 0, 'init_motor': 0, 'monitoring': 0}
        latencies = []

        print(f"[Throughput] Starting {n_seconds}s throughput test...")

        while time.time() < end_time:
            msg, msg_type = get_random_message()
            if msg == "done":
                break
            msg_start = time.time()

            try:
                response = self.send_telnet(self.main_sock, msg, blocking=False)
                msg_end = time.time()

                latencies.append(msg_end - msg_start)
                success_count += 1
                counts_by_type[msg_type] += 1
            except Exception as e:
                error_count += 1
                print(f"[Throughput] Error: {e}")

            total_count += 1

        elapsed = time.time() - start_time
        avg_latency = sum(latencies) / len(latencies) if latencies else 0
        min_latency = min(latencies) if latencies else 0
        max_latency = max(latencies) if latencies else 0

        results = {
            'duration_seconds': elapsed,
            'total_messages': total_count,
            'successful_pairs': success_count,
            'errors': error_count,
            'messages_per_second': success_count / elapsed if elapsed > 0 else 0,
            'avg_latency_ms': avg_latency * 1000,
            'min_latency_ms': min_latency * 1000,
            'max_latency_ms': max_latency * 1000,
            'counts_by_type': counts_by_type,
        }

        print(f"\n[Throughput] Results:")
        print(f"  Duration: {elapsed:.2f}s")
        print(f"  Successful message/response pairs: {success_count}")
        print(f"  Errors: {error_count}")
        print(f"  Throughput: {results['messages_per_second']:.2f} msg/s")
        print(f"  Latency (avg/min/max): {avg_latency*1000:.1f}ms / {min_latency*1000:.1f}ms / {max_latency*1000:.1f}ms")
        print(f"  By type: {counts_by_type}")

        return results



# -------------------------
# Main script
# -------------------------
if __name__ == "__main__":
    client = SocketTelnetClient()
    client.init_motor()
    time.sleep(1)
    client.start_monitoring()
    client.send_telnet(client.main_sock, f"STP 500", blocking=True)
    client.send_telnet(client.main_sock, f"MOV X {client.zero_pose - 100}", blocking=True)
    client.send_telnet(client.main_sock, f"STP 0", blocking=True)


    input("Press Enter to start test moves...")


    # client.send_telnet(client.main_sock, f"MOV X {client.zero_pose - 1900}", blocking=False)
    # time.sleep(1)
    # client.send_telnet(client.main_sock, f"VEL 0", blocking=False)

    # client.measure_throughput(5)
    import numpy as np

    low, high = 0.1, 1.9


    t = np.linspace(0, 1, 1000)
    

    # Smooth ramp up then ramp down (cosine easing): starts at low, peaks at high (middle), ends at low

    v_range = np.concatenate((
        np.linspace(0.0, 1.0, 250, endpoint=False),
        np.linspace(1.0, 0.0, 250, endpoint=True),
    ))
    v_range *= 500
    print(v_range)

    y_range = low + (high - low) * 0.5 * (1 - np.cos(2 * np.pi * t))
    y_range = y_range[:len(y_range)//2]
    print(len(v_range))
    print(len(y_range))
    assert len(y_range) == len(v_range)
    for y,v in zip(y_range, v_range):
        target = y*1000
        print(target)
        if y == y_range[-1]:
            client.send_telnet(client.main_sock, f"STP 500", blocking=True)
        # client.send_telnet(client.main_sock, f"ABORT 0", blocking=False)
        client.send_telnet(client.main_sock, f"VEL {v}", blocking=False)
        # print(f"s{}")
        client.send_telnet(client.main_sock, f"MOV X {client.zero_pose - target}", blocking=False)
        
        time.sleep(0.01)

    # client.send_telnet(client.main_sock, f"MOV X {client.zero_pose - 1800}", blocking=False)
    # time.sleep(1)
    # print("SENDING NEXT")
    # client.send_telnet(client.main_sock, f"MOV X {client.zero_pose - 100}", blocking=False)

    # time.sleep(2)
    # 
    # input("press Enter to continue...")
    # client.measure_throughput(10)
    # time.sleep(3)
    # inp = ""
    # try:
    #     while inp.lower() != "q":
    #         valid_input = False
    #         while not valid_input:
    #             inp = input("\nEnter user_units to move to (or 'q' to quit): ")
    #             if inp.lower() == "q" or inp.isnumeric():
    #                 valid_input = True
    #             else:
    #                 print("Invalid input")
                
    #         if inp.lower() != "q":                
    #             resp = client.goto_pose(inp)
                
    #             # Wait for movement to complete
    #         while client.is_moving:
    #             time.sleep(0.01)
    #         print("Movement complete!")
    # except KeyboardInterrupt:
    #     print("\nInterrupted by user")
    # except Exception as e:
    #     print(f"Error during command execution: {e}")
    # finally:
    #     client.close()
