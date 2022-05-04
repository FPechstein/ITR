import rclpy
import time
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.right_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.t_time=time.time()
        self.t_time2=0
        self.end_time=0
        self.flag=0
        self.z=0.35
        self.timer=140
        
    def timer_cb(self):
    	
        msg = Twist()
        self.flag=0
        #self.get_logger().info({x})
        self.t_time2=time.time()
        self.end_time=self.t_time2-self.t_time
        self.get_logger().info(f'self.timer: {self.timer}')
        
        if self.end_time >= self.timer:
        	self.z=self.z+0.3
        	self.t_time=time.time()
        	self.timer=self.timer*0.75
        	
        y = self.right_distance -self.z + 0.05
        x = self.forward_distance - self.z
        
        if x > 0.14:
        	msg.linear.x = 0.12
        	self.publisher.publish(msg)
        	self.get_logger().info('move no obj')
        if x <= 0.14 and y>0.12:
        	self.flag=1
        	msg.angular.z= -0.25
        	self.publisher.publish(msg)
        	self.get_logger().info('move cause objective infront')
	if x <= 0.14 and y<0.12:
		self.flag=1
		msg.angular.z=0.25
		self.publisher.publish
        if y <= 0.1 and self.flag != 1:
        	self.get_logger().info('right side too close')
        	msg.angular.z=0.04
        	self.publisher.publish(msg)
        	
        if y >0.15 and self.flag != 1:
        	msg.angular.z= -0.2
        	self.get_logger().info('right side far but not too far away')
        	self.publisher.publish(msg)


    def laser_cb(self, msg):
        self.forward_distance =min(msg.ranges[:30]+msg.ranges[-30:])
        self.right_distance= min(msg.ranges[:80]+msg.ranges[110:])
	
	
	
	 


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
