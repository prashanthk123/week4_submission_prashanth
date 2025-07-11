#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import math
from std_msgs.msg import Float64MultiArray

link1=2.0
link2=1.5

class EndEffectorControl(Node):

    def __init__(self):
        super().__init__("inverse_kinematics")
        self.currentz=3.5
        self.currentx=0
        self.pub_joint_angles=self.create_publisher(Float64MultiArray,"/joint_angles_goal",10)
        self.state_sub=self.create_subscription(Point,"/end_effector_position",self.update_coordinate,10)

    def update_coordinate(self,msg:Point):
        self.currentz=msg.z
        self.currentx=msg.x
        goalz=msg.z
        goalx=msg.x
        joint_angles=Float64MultiArray()
        input_axis=input("Enter which axis to move the end effector on(x or z):")
        if(input_axis!='x'and input_axis!='z'):
            self.get_logger().info("invalid axis")
            return
        
        input_distance=input("Enter distance to move end effector(max 0.5):")
        if(abs(float(input_distance))>0.5):
            self.get_logger().info("invalid distance")
            return
        
        if(input_axis=='x'): goalx=self.currentx+float(input_distance)
        else: goalz=self.currentz+float(input_distance)

        result=self.calculate_angles(goalz,goalx)
        if(result!=None):
            
            joint_angles.data=[result[0],result[1]]
            self.pub_joint_angles.publish(joint_angles)

        return
        
        
    def calculate_angles(self,goalz,goalx):
        
        a1=((goalz**2+goalx**2)-(link1**2+link2**2))/(2.0*link2*link1)
        if(abs(a1)<=1):angle1=math.acos(a1)
        else:
            self.get_logger().info("Unreachable point")
            return None
        
        angle2=math.atan2(goalx,goalz)-math.atan2(link2*math.sin(angle1),link1+link2*math.cos(angle1))
        return (angle1,angle2)



def main(args=None):
    rclpy.init(args=args)
    node=EndEffectorControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()