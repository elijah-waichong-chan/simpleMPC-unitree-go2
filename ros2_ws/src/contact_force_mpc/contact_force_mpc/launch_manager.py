#!/usr/bin/env python3

import os
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class LaunchManager(Node):
    def __init__(self) -> None:
        super().__init__("launch_manager")

        self.declare_parameter("mujoco_package", "contact_force_mpc")
        self.declare_parameter("mujoco_launch_file", "mujoco_robot.launch.py")
        self.declare_parameter("control_stack_package", "contact_force_mpc")
        self.declare_parameter("control_stack_launch_file", "control_stack.launch.py")

        self._mujoco_proc: Optional[subprocess.Popen] = None
        self._control_proc: Optional[subprocess.Popen] = None

        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub_mujoco_launch_status = self.create_publisher(
            Bool, "/status/mujoco/launch_running", status_qos
        )
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.srv_start = self.create_service(Trigger, "/launch/mujoco_robot/start", self._handle_start_mujoco)
        self.srv_stop = self.create_service(Trigger, "/launch/mujoco_robot/stop", self._handle_stop_mujoco)
        self.srv_start_ctrl = self.create_service(Trigger, "/launch/control_stack/start", self._handle_start_control)
        self.srv_stop_ctrl = self.create_service(Trigger, "/launch/control_stack/stop", self._handle_stop_control)

    def _is_proc_running(self, proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def _publish_status(self) -> None:
        msg = Bool()
        msg.data = self._is_proc_running(self._mujoco_proc)
        self._pub_mujoco_launch_status.publish(msg)

    def _handle_start_mujoco(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._mujoco_proc is not None and self._mujoco_proc.poll() is None:
            response.success = False
            response.message = "mujoco_robot launch already running"
            self._publish_status()
            return response

        pkg = str(self.get_parameter("mujoco_package").value)
        launch_file = str(self.get_parameter("mujoco_launch_file").value)

        cmd = ["ros2", "launch", pkg, launch_file]
        try:
            self._mujoco_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            response.success = False
            response.message = f"failed to start: {exc}"
            self._publish_status()
            return response

        response.success = True
        response.message = f"started: {' '.join(cmd)}"
        self._publish_status()
        return response

    def _handle_stop_mujoco(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._mujoco_proc is None or self._mujoco_proc.poll() is not None:
            response.success = False
            response.message = "mujoco_robot launch not running"
            self._publish_status()
            return response

        try:
            os.killpg(os.getpgid(self._mujoco_proc.pid), signal.SIGINT)
            self._mujoco_proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self._mujoco_proc.pid), signal.SIGTERM)
                self._mujoco_proc.wait(timeout=5.0)
            except Exception as exc:
                response.success = False
                response.message = f"failed to stop: {exc}"
                return response
        finally:
            self._mujoco_proc = None

        response.success = True
        response.message = "stopped mujoco_robot launch"
        self._publish_status()
        return response

    def _handle_start_control(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._control_proc is not None and self._control_proc.poll() is None:
            response.success = False
            response.message = "control_stack launch already running"
            return response

        pkg = str(self.get_parameter("control_stack_package").value)
        launch_file = str(self.get_parameter("control_stack_launch_file").value)

        cmd = ["ros2", "launch", pkg, launch_file]
        try:
            self._control_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            response.success = False
            response.message = f"failed to start: {exc}"
            return response

        response.success = True
        response.message = f"started: {' '.join(cmd)}"
        return response

    def _handle_stop_control(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._control_proc is None or self._control_proc.poll() is not None:
            response.success = False
            response.message = "control_stack launch not running"
            return response

        try:
            os.killpg(os.getpgid(self._control_proc.pid), signal.SIGINT)
            self._control_proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self._control_proc.pid), signal.SIGTERM)
                self._control_proc.wait(timeout=5.0)
            except Exception as exc:
                response.success = False
                response.message = f"failed to stop: {exc}"
                return response
        finally:
            self._control_proc = None

        response.success = True
        response.message = "stopped control_stack launch"
        return response

    def _terminate_proc(self, proc: Optional[subprocess.Popen]) -> bool:
        if proc is None or proc.poll() is not None:
            return False
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5.0)
            except Exception:
                return False
        return True

    def stop_all(self) -> None:
        if self._terminate_proc(self._mujoco_proc):
            self.get_logger().info("Stopped mujoco_robot launch (shutdown).")
        if self._terminate_proc(self._control_proc):
            self.get_logger().info("Stopped control_stack launch (shutdown).")
        self._mujoco_proc = None
        self._control_proc = None


def main() -> None:
    rclpy.init()
    node = LaunchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_all()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
