
import rclpy

from cv_node.cv_publisher import SimplePublisher
from cv_node.cv_subscriber import SimpleSubscriber

from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    
    rclpy.init(args=args)
    
    try:
        simple_publisher = SimplePublisher()
        simple_subscriber = SimpleSubscriber()

        executor = SingleThreadedExecutor()
        executor.add_node(simple_publisher)
        executor.add_node(simple_subscriber)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            
            simple_publisher.destroy_node()
            simple_subscriber.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
