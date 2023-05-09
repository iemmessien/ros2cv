# Run more than one node at the same time by composing them with a single executor

# Import the necessary libraries
import rclpy

from vision_python_package.vision_publisher import VisionPublisher      # Import the publisher class
from vision_python_package.vision_subscriber import VisionSubscriber    # Import the subscriber class

from rclpy.executors import SingleThreadedExecutor          # Import the single threaded executor class


def main(args=None):
    
    # Initialize the ROS Client Library for Python
    rclpy.init(args=args)
    
    try:
        # Create two nodes which are objects or instances of the VisionPublisher and VisionSubscriber class
        my_v_publisher = VisionPublisher()
        my_v_subscriber = VisionSubscriber()

        # Create an executor object and add the two nodes to the object
        executor = SingleThreadedExecutor()
        executor.add_node(my_v_publisher)
        executor.add_node(my_v_subscriber)

        try:
            # Instead of spinning the main executable directly, 
            # we spin the executor that contains our two nodes.

            # Spin the executor that contains our two nodes
            # Spin the nodes so that both callback functions of the node classes are called
            executor.spin()
        
        finally:
            # Shutdown the executor
            executor.shutdown()
            
            # Destroy the nodes explicitly
            my_v_publisher.destroy_node()
            my_v_subscriber.destroy_node()

    finally:
        # Shutdown the ROS Client Library for Python
        rclpy.shutdown()


if __name__ == '__main__':
    main()
