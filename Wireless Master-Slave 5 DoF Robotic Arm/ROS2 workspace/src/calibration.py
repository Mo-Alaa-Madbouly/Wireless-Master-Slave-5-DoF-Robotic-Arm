import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import time

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
        print("✅ Connected to Arduino!")
        print("=== Calibration Mode ===")
        print("هتدخل زاوية من 0 لـ 180 لكل servo وبتشوف الحركة")

    def test_servo(self, joint_num, angle):
        angles = [90, 90, 90, 90, 90]  # كل الـ servos في المنتصف
        angles[joint_num - 1] = angle   # غير الـ joint اللي بتتستيه
        data = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]},{angles[4]}\n"
        self.arduino.write(data.encode())
        print(f"✅ Joint{joint_num} → {angle}°")

def main():
    rclpy.init()
    node = CalibrationNode()

    while True:
        print("\n--- اختار الـ Joint ---")
        print("1: joint1  2: joint2  3: joint3  4: joint4  5: joint5  0: خروج")
        
        try:
            joint = int(input("Joint number: "))
            if joint == 0:
                break
            if joint < 1 or joint > 5:
                print("❌ رقم غلط!")
                continue
                
            angle = int(input(f"Angle for joint{joint} (0-180): "))
            node.test_servo(joint, angle)
            
        except ValueError:
            print("❌ ادخل رقم صح!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()