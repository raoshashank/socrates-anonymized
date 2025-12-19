#!/usr/bin/env python3
import rclpy
import rclpy.logging
from hunav_msgs.msg import Agents,Agent
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import UInt32
class SimpleGestureListener(Node):
    def __init__(self):
        super().__init__('simple_gesture_listener')
        qos = QoSProfile(depth = 10)
        self.robot_gesture_sub = self.create_subscription(UInt32,'/robot_gesture',self.robotGestCallback,qos)
        self.human_sub = self.create_subscription(Agents,'/human_states',self.humanStateCallback,qos)
        self.robot_dict = {0:"NOTHING", 1:"WAIT", 2:"PROCEED",3:"ACKNOWLEDGED",4:"EXCUSE ME"}
        self.human_dict = {0:"NOTHING", 1:"WAIT", 2:"PROCEED",3:"EXCUSE ME"}
    def robotGestCallback(self,msg):
        if msg.data!=0:
            gstr = f'Robot says {self.robot_dict[msg.data]}( {msg.data})'
            self.get_logger().info(gstr)        
        
    def humanStateCallback(self,msg):
        for agent in msg.agents:
            if agent.gesture!=0:
                gstr = f'Agent {agent.name} says {self.human_dict[agent.gesture]}({agent.gesture})'
                self.get_logger().info(gstr,throttle_duration_sec = 2.0)

if __name__ == '__main__':
    rclpy.init()
    simplegst=SimpleGestureListener()
    rclpy.spin(simplegst)
    simplegst.destroy_node()
    rclpy.shutdown()