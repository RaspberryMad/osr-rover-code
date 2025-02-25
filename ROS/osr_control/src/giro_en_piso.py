#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import board, busio, adafruit_bno055
from osr_interfaces.msg import CommandDrive, CommandCorner

class CmdSender(Node):
    def __init__(self):
        super().__init__('cmd_sender')
        self.corner_pub = self.create_publisher(CommandCorner, '/cmd_corner', 10)
        self.drive_pub  = self.create_publisher(CommandDrive, '/cmd_drive', 10)
        self.timer      = self.create_timer(0.1, self.loop)
        self.sensor     = adafruit_bno055.BNO055_I2C(busio.I2C(board.SCL, board.SDA))
        self.last_yaw   = None
        self.cumulative_angle = 0.0
        self.turning    = True

    def angle_diff(self, current, previous):
        diff = current - previous
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    def loop(self):
        euler = self.sensor.euler
        if euler is None or euler[0] is None:
            self.get_logger().warning("Error de lectura del sensor")
            return
        yaw = euler[0]
        if self.last_yaw is None:
            self.last_yaw = yaw
            self.send_turn_command()
            return
        diff = abs(self.angle_diff(yaw, self.last_yaw))
        self.cumulative_angle += diff
        self.last_yaw = yaw
        self.get_logger().info(f"Rotación acumulada: {self.cumulative_angle:.2f}°")
        if self.cumulative_angle >= 350 and self.turning:
            self.send_stop_command()
            self.turning = False
        elif self.turning:
            self.send_turn_command()

    def send_turn_command(self):
        corner = CommandCorner(
            left_front_pos=0.52, left_back_pos=-0.52,
            right_back_pos=0.52, right_front_pos=-0.52
        )
        drive  = CommandDrive(
            left_front_vel=2.0, left_middle_vel=2.0, left_back_vel=2.0,
            right_front_vel=2.0, right_middle_vel=2.0, right_back_vel=2.0
        )
        self.corner_pub.publish(corner)
        self.drive_pub.publish(drive)

    def send_stop_command(self):
        corner = CommandCorner(
            left_front_pos=0.0, left_back_pos=0.0,
            right_back_pos=0.0, right_front_pos=0.0
        )
        drive  = CommandDrive(
            left_front_vel=0.0, left_middle_vel=0.0, left_back_vel=0.0,
            right_front_vel=0.0, right_middle_vel=0.0, right_back_vel=0.0
        )
        self.corner_pub.publish(corner)
        self.drive_pub.publish(drive)
        self.get_logger().info("360° completado. Robot detenido.")
        # Detenemos ROS y salimos del programa
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = CmdSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
