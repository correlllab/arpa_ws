#!/usr/bin/env python3
import socket
import time
import threading


# -------------------------
# Default settings
# -------------------------
DEFAULT_HOST = "192.168.100.1"
DEFAULT_PORT = 5002
TIMEOUT = 5                  # Socket timeout in seconds
DELAY_BETWEEN_CMDS = 0.2     # seconds to wait after each command
POSITION_CHECK_INTERVAL = 0.1  # seconds between position checks
MOVEMENT_THRESHOLD = 0.0001   # minimum change to consider as movement
ENCODER_0_READING = -517891070
ENCODER_PPU = 26214.4
STATIONARY_THRESHOLD = 5  # Number of consecutive stationary readings
MIN_POSITION_MM = 100.0  # Minimum valid position in mm
MAX_POSITION_MM = 2000.0  # Maximum valid position in mm



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
        
    def send_telnet(self, sock: socket.socket, message: str) -> list[str]:
        """
        Send a string message over the specified socket and read the response.
        Ensures the message ends with CRLF before sending.
        Returns the response as a list of strings (lines).
        """
        if not message.endswith("\r\n"):
            message = message + "\r\n"
        sock.sendall(message.encode("ascii"))

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
        # print(f"[Init motor]{drive_response}")

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

                # Only print when moving
                if self.is_moving:
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

# -------------------------
# Main script
# -------------------------
if __name__ == "__main__":
    client = SocketTelnetClient()
    client.init_motor()
    client.start_monitoring()
    time.sleep(3)
    inp = ""
    try:
        while inp.lower() != "q":
            valid_input = False
            while not valid_input:
                inp = input("\nEnter user_units to move to (or 'q' to quit): ")
                if inp.lower() == "q" or inp.isnumeric():
                    valid_input = True
                else:
                    print("Invalid input")
                
            if inp.lower() != "q":                
                resp = client.goto_pose(inp)
                
                # Wait for movement to complete
            while client.is_moving:
                time.sleep(0.01)
            print("Movement complete!")
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error during command execution: {e}")
    finally:
        client.close()
