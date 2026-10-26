import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class GoalSubscriberNode(Node):
    def __init__(self, navigator):
        super().__init__('coord_to_nav_node')
        self.navigator = navigator
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info('Nav_to_pose Node initialized. Listening on /goal_pose.')

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received Goal Coordinates: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )
        
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        self.navigator.goToPose(msg)

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    
    node = GoalSubscriberNode(navigator)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()