#!/usr/bin/env python3
"""
Nodo de teleoperación para el JPL Open Source Rover Project.
Controla el rover mediante el teclado:
    w : avanzar (velocidad 0.1 m/s)
    s : retroceder (velocidad -0.1 m/s)
    a : girar a la izquierda (angular -0.1 rad/s + velocidad mínima 0.05 m/s)
    d : girar a la derecha (angular 0.1 rad/s + velocidad mínima 0.05 m/s)
    <espacio> : parar (0 m/s y 0 rad/s)

Nota: Este script utiliza las librerías 'termios' y 'tty' para leer el teclado en modo raw.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopRover(Node):
    def __init__(self):
        super().__init__('teleop_rover')
        # Publicador de mensajes Twist en el tópico que usa el nodo principal del rover
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 1)

        # Parámetros de velocidad
        self.speed_linear = 0.15    # Velocidad normal para avanzar
        self.ratio_angular = 0.15   # Velocidad angular

        # Guarda la configuración original de la terminal
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "Nodo Teleop Rover iniciado. Usa WASD para moverte y <espacio> para parar. Ctrl-C para salir.")

    def get_key(self):
        """Lee una tecla del teclado en modo no bloqueante."""
        tty.setraw(sys.stdin.fileno())
        # Espera 0.1 segundos para ver si hay entrada
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()

                if key == 'w':
                    twist.linear.x = self.speed_linear
                    twist.angular.z = 0.0
                elif key == 's':
                    twist.linear.x = -self.speed_linear
                    twist.angular.z = 0.0
                elif key == 'q':
                    twist.linear.x = self.speed_linear
                    twist.angular.z = self.ratio_angular
                elif key == 'e':
                    twist.linear.x = self.speed_linear
                    twist.angular.z = -self.ratio_angular
                elif key == 'a':
                    twist.linear.x = -self.speed_linear
                    twist.angular.z = self.ratio_angular
                elif key == 'd':
                    twist.linear.x = -self.speed_linear
                    twist.angular.z = -self.ratio_angular
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':  # Ctrl-C
                    break
                else:
                    continue

                # Publica el mensaje Twist
                self.publisher_.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error en teleoperación: {e}")
        finally:
            # Al salir, enviamos un comando de paro para garantizar que el rover se detenga
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info("Nodo Teleop Rover finalizado.")


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopRover()

    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
