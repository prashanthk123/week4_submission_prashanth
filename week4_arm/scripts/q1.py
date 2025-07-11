#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import math

link1=2.0
link2=1.5

class EndEffectorPosition(Node):

    def __init__(self):
        super().__init__("forward_kinematics")
        self.pub_coords=self.create_publisher(Point,"/end_effector_position",10)
        self.state_sub=self.create_subscription(JointState,"/joint_states",self.find_coordinate,10)

    def find_coordinate(self,msg:JointState):
        coordinate=Point()
        shoulder_angle=msg.position[1]
        elbow_angle=msg.position[2]
        z=link2*(math.cos(shoulder_angle+elbow_angle))+link1*(math.cos(shoulder_angle))
        x=link2*(math.sin(shoulder_angle+elbow_angle))+link1*(math.sin(shoulder_angle))
        coordinate.z=z
        coordinate.x=x
        self.pub_coords.publish(coordinate)
        self.get_logger().info("current coords:"+str((x,0,z)))
        


def main(args=None):
    rclpy.init(args=args)
    node=EndEffectorPosition()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()