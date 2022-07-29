# Import the necessary libraries
import rclpy						# ROS 2 Client Library for Python
from rclpy.node import Node			# Inherit from the Node Super Class to handle the creation of nodes

from std_msgs.msg import String		# For a message type of String
from sensor_msgs.msg import Image	# For a message type of Image

from cv_bridge import CvBridge 		# Package for converting between ROS 2 and OpenCV Images
import cv2 							# The OpenCV library


class SimpleSubscriber(Node):
	# Create a subscriber class which is a subclass of the Node super class
	
	def __init__(self):
		# >> Class Constructor to set up the node
		# Initiate the Node class's constructor and give it a name
		super().__init__('simple_subscriber')
		
		# Create the subscriber
		self.subscription = self.create_subscription(
			Image,
			'video_frames',
			self.listener_callback,
			10)
		self.subscription	# Prevent unused variable warning
		
		# Used to convert between ROS 2 and OpenCV images
		self.br = CvBridge()
		
	def listener_callback(self, data):
		# >> A Callback function
		# This function gets called at the specified time period
		
		# Display the message on the console
		self.get_logger().info('Receiving...')
		
		# Retrieve the image data and the optional message
		# The 'mgmsg_to_cv2' method converts a ROS 2
		# image message to an OpenCV image
		current_frame = self.br.imgmsg_to_cv2(data)
		
		# Display the image data
		cv2.imshow("camera", current_frame)
		
		cv2.waitKey(1)
			
def main(args=None):
	
	# Initialize the rclpy library
	rclpy.init(args=args)
	
	# Create the node which is an object or instance of SimpleSubscriber
	my_simple_subscriber = SimpleSubscriber()
	
	# Spin the node so the callback function is called
	rclpy.spin(my_simple_subscriber)
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	my_simple_subscriber.destroy_node()
	
	# Shutdown the ROS 2 Client Library for Python
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
	
