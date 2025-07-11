#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import math

link1=2.0
link2=1.5


# DOES NOT WORK - WRONG EQUATION


class ThreeDoFEndEffectorPosition(Node):

    def __init__(self):
        super().__init__("position_3D")
        self.pub_coords=self.create_publisher(Point,"/end_effector_position_3D",10)
        self.state_sub=self.create_subscription(JointState,"/joint_states",self.find_coordinate_3D,10)

    def find_coordinate_3D(self,msg:JointState):
        coordinate=Point()
        shoulder_angle=msg.position[1]
        elbow_angle=msg.position[2]
        base_yaw=msg.position[0]
        sine1=math.sin(base_yaw)
        sine2=math.sin(shoulder_angle)
        sine3=math.sin(elbow_angle)
        cos1=math.cos(base_yaw)
        cos2=math.cos(shoulder_angle)
        cos3=math.cos(elbow_angle)


        x=link2*cos1*(cos2*cos3+sine2*sine3) + link1*cos1*cos2
        
        y=link2*sine1*(cos2*cos3+sine2*sine3) + link1*cos2*sine1
        
        z=link2*(sine3*cos2 - cos3*sine2) + link1*sine2


        coordinate.x=x
        coordinate.y=y
        coordinate.z=z
        self.pub_coords.publish(coordinate)
        self.get_logger().info("current coords:"+str((x,y,z)))
        self.get_logger().info("current angles:"+str((base_yaw,shoulder_angle,elbow_angle)))
        


def main(args=None):
    rclpy.init(args=args)
    node=ThreeDoFEndEffectorPosition()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()