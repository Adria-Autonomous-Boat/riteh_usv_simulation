#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64
from actuator_msgs.msg import Actuators
from px4_msgs.msg import ActuatorServos



class EnginesControllerSim(Node):

    def __init__(self):
        super().__init__('engines_controller_sim')

        self.declare_parameter('left_thruster', '/usv/thrusters/left/thrust')
        self.declare_parameter('right_thruster', '/usv/thrusters/right/thrust')

        self.declare_parameter('neutral', 700)
        self.declare_parameter('forward_multiplier', 600)
        self.declare_parameter('reverse_multiplier', 200)

        self.declare_parameter('gz_thruster_max_ang_vel', 2000)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Motor ESC settings
        self.NEUTRAL = \
            self.get_parameter('neutral').get_parameter_value().integer_value
        self.FORWARD_MULTIPLIER = \
            self.get_parameter('forward_multiplier').get_parameter_value().integer_value
        self.REVERSE_MULTIPLIER = \
            self.get_parameter('reverse_multiplier').get_parameter_value().integer_value

        self.GZ_THRUSTER_MAX_ANG_VEL = self.get_parameter('gz_thruster_max_ang_vel').get_parameter_value().integer_value

        self.actuator_servos = self.create_subscription(
            ActuatorServos,
            '/actuator_servos',
            self.engines_velocity_callback,
            qos_profile=qos_profile
        )

        self.right_engine_publisher = self.create_publisher(
            Float64, 
            self.get_parameter('right_thruster').value, 
            10
        )

        self.left_engine_publisher = self.create_publisher(
            Float64, 
            self.get_parameter('left_thruster').value, 
            10
        )


    def px4_engines_velocity_callback(self, msg: Actuators):
        left_vel = self.normalizeMotorCommand(msg.velocity[0])
        right_vel = self.normalizeMotorCommand(msg.velocity[1])

        self.gz_send_engine_commands(left_vel, right_vel)


    def engines_velocity_callback(self, msg: ActuatorServos):
        left_vel = self.calculateMotorCommand(msg.control[0])
        right_vel = self.calculateMotorCommand(msg.control[1])

        self.gz_send_engine_commands(left_vel, right_vel)


    def gz_send_engine_commands(self, left_vel: float, right_vel: float):
        left_engine_cmd = Float64()
        right_engine_cmd = Float64()

        left_engine_cmd.data = (left_vel)
        right_engine_cmd.data = (right_vel)


        self.left_engine_publisher.publish(left_engine_cmd)
        self.right_engine_publisher.publish(right_engine_cmd)


    def normalizeMotorCommand(self, speed):
        if (speed < self.NEUTRAL):
            # Motor forward
            return ((self.NEUTRAL - speed) / self.FORWARD_MULTIPLIER) * self.GZ_THRUSTER_MAX_ANG_VEL
        elif (speed > self.NEUTRAL):
            # Motor backwards
            return ((self.NEUTRAL - speed) / self.REVERSE_MULTIPLIER) * self.GZ_THRUSTER_MAX_ANG_VEL
        
        return 0.0
    
    def calculateMotorCommand(self, speed_norm):
        if (speed_norm > 0.0):
            # Motor forward
            return float(speed_norm * self.GZ_THRUSTER_MAX_ANG_VEL)
        elif (speed_norm < 0.0):
            # Motor backwards
            return float(speed_norm * self.GZ_THRUSTER_MAX_ANG_VEL)
        
        return 0.0



def main(args=None):
    rclpy.init(args=args)

    engines_controller_sim = EnginesControllerSim()
    rclpy.spin(engines_controller_sim)
    rclpy.shutdown()

if __name__ == '__main__':
    main()