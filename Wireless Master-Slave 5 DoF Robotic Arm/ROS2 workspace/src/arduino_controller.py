import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
import serial
import math
import time

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Connected to Arduino!')
        except Exception as e:
            self.get_logger().error(f'Cannot connect to Arduino: {e}')
            return

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arduino_arm_controller/follow_joint_trajectory',
            self.execute_callback)

        self._gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            self.gripper_callback)

        self.last_j1 = 90
        self.last_j2 = 90
        self.last_j3 = 90
        self.last_j4 = 90
        self.last_j5 = 90

        self.get_logger().info('Arduino Controller Started!')

    def convert(self, rad, reverse=False, offset=0):
        degrees = math.degrees(rad)
        if reverse:
            degrees = -degrees
        servo_angle = int(degrees + 90 + offset)
        return max(0, min(180, servo_angle))

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        last_point = trajectory.points[-1]
        joint_angles = dict(zip(joint_names, last_point.positions))

        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

        self.get_logger().info(f'Raw joint1: {joint_angles.get("joint1", 0.0)} rad = {math.degrees(joint_angles.get("joint1", 0.0))} deg')

        j1 = self.convert(joint_angles.get('joint1', 0.0), reverse=False)
        j2 = self.convert(joint_angles.get('joint2', 0.0), reverse=False, offset=-40)
        j3 = self.convert(joint_angles.get('joint3', 0.0), reverse=True, offset=-30)
        j4 = self.convert(joint_angles.get('joint4', 0.0), reverse=False, offset=-70)

        self.last_j1 = j1
        self.last_j2 = j2
        self.last_j3 = j3
        self.last_j4 = j4

        data = f"{j1},{j2},{j3},{j4},{self.last_j5}\n"
        self.arduino.write(data.encode())
        self.get_logger().info(f'Sent: {data.strip()}')

        time.sleep(max(duration, 2.0))

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    def gripper_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        last_point = trajectory.points[-1]
        joint_angles = dict(zip(joint_names, last_point.positions))

        j5 = int(math.degrees(joint_angles.get('joint5', 0.0)))
        j5 = max(0, min(90, j5))
        self.last_j5 = j5

        data = f"{self.last_j1},{self.last_j2},{self.last_j3},{self.last_j4},{j5}\n"
        self.arduino.write(data.encode())
        self.get_logger().info(f'Gripper Sent: {data.strip()}')

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

def main():
    rclpy.init()
    node = ArduinoController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()