import time
import math
import rclpy
from rclpy.node import Node
from osr_interfaces.msg import CommandDrive, CommandCorner

class RoverSteering(Node):
    def __init__(self):
        super().__init__('rover_steering')
        self.drive_pub = self.create_publisher(CommandDrive, '/cmd_drive', 10)
        self.corner_pub = self.create_publisher(CommandCorner, '/cmd_corner', 10)
        
        # Ajusta según tu rover:
        self.d2 = 0.30  # Distancia desde el centro del rover a eje delantero/trasero
        self.d1 = 0.18  # Mitad del ancho (track/2)
        
        # Velocidades típicas:
        self.forward_speed = 1.0

    def center_servos(self):
        """Ruedas alineadas a 0 rad (completamente rectas)."""
        cmd = CommandCorner(
            right_back_pos=0.0,
            right_front_pos=0.0,
            left_front_pos=0.0,
            left_back_pos=0.0
        )
        self.get_logger().info('Centering servos')
        self.corner_pub.publish(cmd)

    def stop(self):
        """Frena el movimiento de traslación."""
        cmd = CommandDrive(
            left_front_vel=0.0,
            left_middle_vel=0.0,
            left_back_vel=0.0,
            right_front_vel=0.0,
            right_middle_vel=0.0,
            right_back_vel=0.0
        )
        self.get_logger().info('Stopping rover')
        self.drive_pub.publish(cmd)

    def drive(self, speed=None):
        """Controla la traslación del rover, permite avanzar (+) o retroceder (-)."""
        if speed is None:
            speed = self.forward_speed
        cmd = CommandDrive(
            left_front_vel=-speed,  # Signo negativo en ruedas izquierdas
            left_middle_vel=-speed,
            left_back_vel=-speed,
            right_front_vel=speed,  # Signo positivo en ruedas derechas
            right_middle_vel=speed,
            right_back_vel=speed
        )
        direction = "backward" if speed < 0 else "forward"
        self.get_logger().info(f'Moving {direction} at speed {abs(speed)}')
        self.drive_pub.publish(cmd)

    def set_corner_angles(self, theta_fl, theta_fr, theta_rl, theta_rr):
        """
        Envía a cada servo de esquina el ángulo en radianes deseado.
        Ajusta signo u offset si tu servo en 0 rad no es “recto” exactamente.
        """
        corner_cmd = CommandCorner(
            right_back_pos= theta_rr,
            right_front_pos=theta_fr,
            left_front_pos= theta_fl,
            left_back_pos=  theta_rl
        )
        self.corner_pub.publish(corner_cmd)

    def execute_turn(self, R, speed, sign):
        """
        Gira el rover con Ackermann, ajustando tanto ángulos como
        velocidades de cada lado para describir arcos correctos.
        """

        # Validar el radio mínimo
        MIN_RADIUS = 0.7
        if R < MIN_RADIUS:
            self.get_logger().warning(
                f'El radio {R:.2f} es menor al mínimo permitido ({MIN_RADIUS:.2f}). '
                f'Ajustando R={MIN_RADIUS:.2f} automáticamente.'
            )
            R = MIN_RADIUS

        if sign == 'left':
            theta_fl = -math.atan(self.d2 / (R + self.d1))
            theta_fr = -math.atan(self.d2 / (R - self.d1))
            theta_rl = -theta_fl
            theta_rr = -theta_fr
        if sign == 'right':
            theta_fr =  math.atan(self.d2 / (R + self.d1))
            theta_fl = math.atan(self.d2 / (R - self.d1))
            theta_rr = -theta_fr
            theta_rl = -theta_fl

        self.set_corner_angles(theta_fl, theta_fr, theta_rl, theta_rr)

        if sign == 'left':
            left_speed  = -speed * (R - self.d1) / R
            right_speed =  speed * (R + self.d1) / R
        if sign == 'right':
            right_speed =  speed * (R - self.d1) / R
            left_speed  = -speed * (R + self.d1) / R

        cmd = CommandDrive(
            left_front_vel = left_speed,
            left_middle_vel= left_speed,
            left_back_vel  = left_speed,
            right_front_vel= right_speed,
            right_middle_vel=right_speed,
            right_back_vel = right_speed
        )
        self.drive_pub.publish(cmd)

        self.get_logger().info(
            f'Ackermann turn with speed: sign={sign}, R={R}, speed={speed:.2f}\n'
            f'   angles: fl={theta_fl:.2f}, fr={theta_fr:.2f}, '
            f'rl={theta_rl:.2f}, rr={theta_rr:.2f}\n'
            f'   velocities: left={left_speed:.2f}, right={right_speed:.2f}'
        )


def main(args=None):
    import time

    rclpy.init(args=args)
    node = RoverSteering()
    
    try:
        node.center_servos()
        
        start_time = time.time()
        
        # Avanzar 
        while time.time() - start_time < 0:
            node.drive(speed=0.5)
        
        # Pausar durante 1 segundo
        node.stop()
        time.sleep(1)

        # Retroceder
        start_time = time.time()
        while time.time() - start_time < 0:
            node.drive(speed=-0.5)
        
        # Girar a la izquierda
        start_time = time.time()
        while time.time() - start_time < 0:
            node.execute_turn(R=0.7, speed=0.5, sign='left')

        # Girar a la derecha
        start_time = time.time()
        while time.time() - start_time < 0:
            node.execute_turn(R=0.7, speed=0.5, sign='right')

        # Detener
        node.stop()
       # node.center_servos()

    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
