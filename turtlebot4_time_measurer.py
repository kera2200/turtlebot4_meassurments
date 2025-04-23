#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import sys
import logging

class Nav2GoalClient(Node):

    def __init__(self):
        super().__init__('nav2_goal_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_time = 0.0
        self.recoveries = 0       
        self.poses = []
        self.current_goal_index = 0
        self.total_goals = 0
        logging.basicConfig(filename='dwb_controller_log_case2.txt',
            filemode='a',
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.INFO
        )
        self.logger = logging.getLogger('Nav2GoalClient')

    def send_goal(self, x, y, yaw=0.0):
        self.get_logger().info(f"Waiting for action server...")
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = 0.0  # No rotation, uses radians
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to x={x}, y={y}")
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # Add feedback callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        navigation_time = feedback.navigation_time
        amount_of_recoveries = feedback.number_of_recoveries
        #self.get_logger().info(f"Navigation time: {navigation_time.sec}s {navigation_time.nanosec}ns")
        self.nav_time = navigation_time.sec + navigation_time.nanosec * 1e-9
        #self.get_logger().info(f"Number of recoveries: {feedback.number_of_recoveries}")
        self.recoveries = amount_of_recoveries
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal reached with result: {result}')
        self.logger.info(f'Goal reached with result: {result}')


        self.get_logger().info(f"Navigation time: {self.nav_time}")
        self.logger.info(f"Navigation time: {self.nav_time}")


        self.get_logger().info(f"Number of recoveries: {self.recoveries}")
        self.logger.info(f"Number of recoveries: {self.recoveries}")
        
        self.total_goals += 1
        self.get_logger().info(f"Finished goals: {self.total_goals}")
        self.logger.info(f"Finished goals: {self.total_goals}")
        if self.total_goals == 6: #Stopping after x amount of goals
            self.get_logger().info("All goals completed.")
            rclpy.shutdown()
            return

        self.current_goal_index = (self.current_goal_index + 1) % len(self.poses)
        target_x, target_y = self.poses[self.current_goal_index]
        self.get_logger().info(f"Sending next goal to x={target_x}, y={target_y}")
        self.send_goal(target_x, target_y)
        
        
      

def main(args=None):
    rclpy.init(args=args)
    client = Nav2GoalClient()
    poses = [(8.0, -8.0), (0.0511, -0.0342)]
    client.poses = poses
    client.current_goal_index = 0
    target_x, target_y = client.poses[client.current_goal_index]
    client.send_goal(target_x, target_y)
    
   # if len(sys.argv) == 3:
  #      target_x = float(sys.argv[1])
 #       target_y = float(sys.argv[2])
 #   client.send_goal(target_x, target_y)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
