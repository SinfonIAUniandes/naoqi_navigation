import rclpy
from rclpy.node import Node
import qi
import argparse
import sys

from std_srvs.srv import Trigger
from naoqi_utilities_msgs.srv import MoveTo, Explore

class NaoqiNavigationNode(Node):
    """
    ROS2 Node to manage navigation functionalities of a NAO robot.
    """
    def __init__(self, session):
        """
        Initializes the node, NAOqi service clients, and ROS2 services.
        """
        super().__init__('naoqi_navigation_node')
        self.get_logger().info("Initializing NaoqiNavigationNode...")

        # --- NAOqi Service Clients ---
        try:
            self.al_motion = session.service("ALMotion")
            self.al_navigation = session.service("ALNavigation")
            self.get_logger().info("NAOqi service clients obtained successfully.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to NAOqi services: {e}")
            sys.exit(1)

        # --- ROS2 Services ---
        # ALMotion
        self.move_to_service = self.create_service(
            MoveTo,
            '~/move_to',
            self.move_to_callback
        )

        # ALNavigation
        self.navigate_to_service = self.create_service(
            MoveTo,
            '~/navigate_to',
            self.navigate_to_callback
        )
        self.start_exploring_service = self.create_service(
            Explore,
            '~/start_exploring',
            self.start_exploring_callback
        )
        self.stop_exploring_service = self.create_service(
            Trigger,
            '~/stop_exploring',
            self.stop_exploring_callback
        )

        self.get_logger().info("Navigation functionalities node is ready.")

    def move_to_callback(self, request, response):
        """
        Callback to move the robot to a relative target.
        """
        try:
            self.get_logger().info(f"Request to move to ({request.x_coordinate}, {request.y_coordinate}, {request.theta_coordinate}).")
            self.al_motion.moveTo(request.x_coordinate, request.y_coordinate, request.theta_coordinate)
        except Exception as e:
            self.get_logger().error(f"Error sending move command: {e}")
        return response

    def navigate_to_callback(self, request, response):
        """
        Callback to make the robot navigate to a specific coordinate in the map.
        """
        try:
            self.get_logger().info(f"Request to navigate to ({request.x_coordinate}, {request.y_coordinate}).")
            # navigateTo does not use theta, so we ignore it.
            self.al_navigation.navigateTo(request.x_coordinate, request.y_coordinate)
        except Exception as e:
            self.get_logger().error(f"Error sending navigation goal: {e}")
        return response

    def start_exploring_callback(self, request, response):
        """
        Callback to start exploration.
        """
        try:
            self.get_logger().info(f"Request to start exploring with a radius of {request.radius} meters.")
            self.al_navigation.explore(request.radius)
        except Exception as e:
            self.get_logger().error(f"Error starting exploration: {e}")
        return response

    def stop_exploring_callback(self, request, response):
        """
        Callback to stop exploration.
        """
        try:
            self.get_logger().info("Request to stop exploration.")
            self.al_navigation.stopExploration()
        except Exception as e:
            self.get_logger().error(f"Error stopping exploration: {e}")
        return response


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On Robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])

    session = qi.Session()
    try:
        session.connect(f"tcp://{parsed_args.ip}:{parsed_args.port}")
    except RuntimeError:
        print(f"Can't connect to Naoqi at ip \"{parsed_args.ip}\" on port {parsed_args.port}.\n"
              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    naoqi_navigation_node = NaoqiNavigationNode(session)

    try:
        rclpy.spin(naoqi_navigation_node)
    except KeyboardInterrupt:
        print("Closing the navigation functionalities node.")
    finally:
        naoqi_navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()