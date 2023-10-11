# my_robot_control/my_robot_control/robot_control_action_client.py
import rclpy
from rclpy.action import ActionClient
from my_robot_msgs.msg import RobotControl
from my_robot_msgs.msg import RobotControlGoal
from my_robot_msgs.msg import RobotControlResult

class RobotControlActionClient:
    def __init__(self):
        self._action_client = ActionClient(
            rclpy.create_node('robot_control_action_client'),
            RobotControl,
            'robot_control'
        )

    def send_robot_control_goal(self, task_type):
        goal_msg = RobotControlGoal()
        goal_msg.task_type = task_type
        goal_handle = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(goal_handle, timeout_sec=10.0)

        if goal_handle.result() is not None:
            result = goal_handle.result().result
            self.result_callback(result)

    def feedback_callback(self, feedback_msg):
        pass

    def result_callback(self, result_msg):
        if result_msg.success:
            print("Task successfully completed!")
        else:
            print("Task failed.")

def main(args=None):
    rclpy.init(args=args)
    action_client = RobotControlActionClient()

    # Example: Send navigation goal
    action_client.send_robot_control_goal(RobotControlGoal.TASK_NAVIGATION)

    # Example: Send inspection goal
    action_client.send_robot_control_goal(RobotControlGoal.TASK_INSPECTION)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
