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
    def __init__(self, session, robot_url=None):
        """
        Initializes the node, NAOqi service clients, and ROS2 services.
        """
        super().__init__('naoqi_navigation_node')
        self.get_logger().info("Initializing NaoqiNavigationNode...")
        self.session = session
        self.robot_url = robot_url

        # --- NAOqi Service Clients ---
        try:
            self._refresh_naoqi_services()
            self.get_logger().info("NAOqi service clients obtained successfully.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to NAOqi services: {e}")
            sys.exit(1)

        # --- ROS2 Services ---
        # ALMotion
        self.move_to_service = self.create_service(
            MoveTo,
            'move_to',
            self.move_to_callback
        )

        # ALNavigation
        self.navigate_to_service = self.create_service(
            MoveTo,
            'navigate_to',
            self.navigate_to_callback
        )
        self.start_exploring_service = self.create_service(
            Explore,
            'start_exploring',
            self.start_exploring_callback
        )
        self.stop_exploring_service = self.create_service(
            Trigger,
            'stop_exploring',
            self.stop_exploring_callback
        )

        self.get_logger().info("Navigation functionalities node is ready.")

    def _ensure_session_connected(self):
        if self.session.isConnected():
            return True
        if not self.robot_url:
            return False
        self.get_logger().warn("NAOqi session is disconnected. Attempting to reconnect...")
        try:
            self.session.close()
        except Exception:
            pass
        try:
            self.session.connect(self.robot_url)
            self.get_logger().info("NAOqi session reconnected.")
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not reconnect NAOqi session: {e}")
            return False

    def _service(self, name):
        if not self._ensure_session_connected():
            raise RuntimeError("NAOqi session is not connected")
        return self.session.service(name)

    def _refresh_naoqi_services(self):
        self.al_motion = self._service("ALMotion")
        self.al_navigation = self._service("ALNavigation")

    def move_to_callback(self, request, response):
        """
        Callback to move the robot to a relative target.
        """
        try:
            self._refresh_naoqi_services()
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
            self._refresh_naoqi_services()
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
            self._refresh_naoqi_services()
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
            self._refresh_naoqi_services()
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
    robot_url = f"tcp://{parsed_args.ip}:{parsed_args.port}"
    try:
        session.connect(robot_url)
    except RuntimeError:
        print(f"Can't connect to Naoqi at ip \"{parsed_args.ip}\" on port {parsed_args.port}.\n"
              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    naoqi_navigation_node = NaoqiNavigationNode(session, robot_url)

    try:
        rclpy.spin(naoqi_navigation_node)
    except KeyboardInterrupt:
        print("Closing the navigation functionalities node.")
    finally:
        naoqi_navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()