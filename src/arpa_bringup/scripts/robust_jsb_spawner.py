#!/usr/bin/env python3
"""
Robust joint_state_broadcaster spawner.

The default ROS 2 Humble spawner crashes with FATAL if load_controller
returns "already loaded" (which happens when the first attempt's response
is slow and the controller was loaded server-side before the client
timed out).  This script handles that race gracefully:

  1. load   → OK or already loaded → continue
  2. configure → OK or already configured → continue
  3. activate via switch_controller
"""

import sys
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    LoadController,
    ConfigureController,
    SwitchController,
    ListControllers,
)


CONTROLLER_NAME = "joint_state_broadcaster"
CM_NODE = "/controller_manager"
TIMEOUT_SEC = 60.0
POLL_SEC = 2.0


class RobustSpawner(Node):
    def __init__(self):
        super().__init__("robust_jsb_spawner")
        self._load_cli = self.create_client(
            LoadController, f"{CM_NODE}/load_controller"
        )
        self._configure_cli = self.create_client(
            ConfigureController, f"{CM_NODE}/configure_controller"
        )
        self._switch_cli = self.create_client(
            SwitchController, f"{CM_NODE}/switch_controller"
        )
        self._list_cli = self.create_client(
            ListControllers, f"{CM_NODE}/list_controllers"
        )

    def _wait_for_service(self, cli, label):
        self.get_logger().info(f"Waiting for {label} ...")
        deadline = time.time() + TIMEOUT_SEC
        while not cli.wait_for_service(timeout_sec=POLL_SEC):
            if time.time() > deadline:
                self.get_logger().fatal(f"Timed out waiting for {label}")
                return False
        return True

    def _call(self, cli, request, label, timeout=30.0):
        future = cli.call_async(request)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() > deadline:
                self.get_logger().error(f"{label}: response timed out")
                return None
        return future.result()

    def _is_loaded(self):
        req = ListControllers.Request()
        res = self._call(self._list_cli, req, "list_controllers")
        if res is None:
            return False
        return any(c.name == CONTROLLER_NAME for c in res.controller)

    def _get_state(self):
        req = ListControllers.Request()
        res = self._call(self._list_cli, req, "list_controllers")
        if res is None:
            return "unknown"
        for c in res.controller:
            if c.name == CONTROLLER_NAME:
                return c.state
        return "unloaded"

    def run(self):
        for cli, label in [
            (self._list_cli, "list_controllers"),
            (self._load_cli, "load_controller"),
            (self._configure_cli, "configure_controller"),
            (self._switch_cli, "switch_controller"),
        ]:
            if not self._wait_for_service(cli, label):
                return 1

        # 1. Load
        state = self._get_state()
        self.get_logger().info(f"{CONTROLLER_NAME} current state: {state}")

        if state == "unloaded":
            req = LoadController.Request()
            req.name = CONTROLLER_NAME
            res = self._call(self._load_cli, req, "load", timeout=30.0)
            if res is None:
                if self._is_loaded():
                    self.get_logger().info("load timed out but controller IS loaded")
                else:
                    self.get_logger().fatal("load timed out and controller not loaded")
                    return 1
            elif not res.ok:
                if self._is_loaded():
                    self.get_logger().info("load returned !ok but controller IS loaded")
                else:
                    self.get_logger().fatal("Failed to load controller")
                    return 1

        # 2. Configure
        state = self._get_state()
        self.get_logger().info(f"{CONTROLLER_NAME} state after load: {state}")

        if state in ("unconfigured", "loaded"):
            req = ConfigureController.Request()
            req.name = CONTROLLER_NAME
            res = self._call(self._configure_cli, req, "configure", timeout=15.0)
            if res is None or not res.ok:
                state = self._get_state()
                if state not in ("inactive", "active"):
                    self.get_logger().fatal(f"configure failed, state={state}")
                    return 1

        # 3. Activate
        state = self._get_state()
        self.get_logger().info(f"{CONTROLLER_NAME} state after configure: {state}")

        if state == "inactive":
            req = SwitchController.Request()
            req.activate_controllers = [CONTROLLER_NAME]
            req.deactivate_controllers = []
            req.strictness = SwitchController.Request.BEST_EFFORT
            res = self._call(self._switch_cli, req, "activate", timeout=15.0)
            if res is None or not res.ok:
                state = self._get_state()
                if state != "active":
                    self.get_logger().fatal(f"activate failed, state={state}")
                    return 1

        state = self._get_state()
        self.get_logger().info(f"{CONTROLLER_NAME} final state: {state}")

        if state == "active":
            self.get_logger().info(
                f"Successfully spawned {CONTROLLER_NAME}"
            )
            return 0
        else:
            self.get_logger().fatal(f"Unexpected final state: {state}")
            return 1


def main():
    rclpy.init()
    node = RobustSpawner()
    try:
        rc = node.run()
    except Exception as e:
        node.get_logger().fatal(f"Exception: {e}")
        rc = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
