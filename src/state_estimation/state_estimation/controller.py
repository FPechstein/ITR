import rclpy
import numpy as np
from rclpy.node import Node
# import time

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan


class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        # self.t_time=time.time()
        # self.t_time2=0
        # self.end_time=0
        self.flag = 0
        self.z = 0.35
        # self.timer=140
        self.right_distance = 0
        self.counter = 0
        self.positions = []
        self.goaldirection = [0, 0]
        self.direction = []
        self.x = 0
        self.winke=-10
        self.drehen=0
        # self.y=0

    def timer_cb(self):
        msg = Twist()
        self.flag = 0
        #print(self.counter)
        # self.get_logger().info({x})
        # self.t_time2=time.time()
        # self.end_time=self.t_time2-self.t_time
        # self.get_logger().info(f'self.timer: {self.timer}')
        self.winkel=-10

        if len(self.direction) >= 1:
            radiance=np.arccos( (self.direction[0] * self.goaldirection[0] + self.goaldirection[1] * self.direction[1]) / (np.sqrt(self.direction[0] ** 2 + self.direction[1] ** 2) * np.sqrt( self.goaldirection[0] ** 2 + self.goaldirection[1] ** 2)))
            self.winkel=np.degrees(radiance)
            if self.winkel >25 and self.winkel <155:
            	self.drehen=2
            self.direction.clear()

        # if self.end_time >= self.timer:
        # self.z=self.z+0.3
        # self.t_time=time.time()
        # self.timer=self.timer*0.75

        # self.y = self.right_distance -self.z + 0.05
        self.x = self.forward_distance - self.z
        self.flag=0


        if self.x > 0.14 and self.drehen<=0:
            #print("komme ich hierhin?")
            self.counter = self.counter + 1
            msg.linear.x = 0.12
            msg.angular.z = 0.0
            self.flag=1

        # self.get_logger().info('move no obj')
        if self.x <= 0.14:
            self.counter = 0
            self.flag=1
            msg.linear.x = 0.0
            msg.angular.z = 0.25
            self.direction.clear()
        if  self.drehen>0 and self.flag==0:
            self.counter=0
            #print("winkel = ",self.winkel)
            #print("drehen = ",self.drehen)
            self.drehen=self.drehen-1
            
            msg.angular.z=0.25

        self.publisher.publish(msg)
        # self.get_logger().info('move cause objective infront')

        # if self.y <= 0.1 and self.flag != 1:
        #	self.counter=0
        #	#self.get_logger().info('right side too close')
        #	msg.angular.z=0.06
        #	self.publisher.publish(msg)
        # if self.y >0.15 and self.flag != 1:
        #	self.counter=0
        #	msg.angular.z= -0.2
        #	#self.get_logger().info('right side far but not too far away')
        #	self.publisher.publish(msg)

    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y

        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal

    def laser_cb(self, msg):
        self.forward_distance = min(msg.ranges[:30] + msg.ranges[-30:])
        # self.right_distance= min(msg.ranges[:80]+msg.ranges[110:])

    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y
        if len(self.positions) >= 3:
            self.positions.pop(0)
        self.positions.append(self.position)

        if len(self.positions) >= 3:
            self.direction.append(self.positions[2][0] - self.positions[0][0])
            self.direction.append(self.positions[2][1] - self.positions[0][1])
            print(self.direction, "=direction")

        if self.counter <= 0:
            self.positions.clear()

        self.goaldirection[0] = self.goal[0] - self.position[0]
        self.goaldirection[1] = self.goal[1] - self.position[1]
        # print(self.goaldirection,"=goaldirection")


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
