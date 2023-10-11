# my_robot_control/my_robot_control/robot_control_action_server.py
import rclpy
from rclpy.action import ActionServer
from my_robot_msgs.msg import RobotControl
from my_robot_msgs.msg import RobotControlGoal
from my_robot_msgs.msg import RobotControlResult

class RobotControlActionServer:
    def __init__(self):
        self._action_server = ActionServer(
            rclpy.create_node('robot_control_action_server'),
            RobotControl,
            'robot_control',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        if goal_handle.request.task_type == RobotControlGoal.TASK_NAVIGATION:
            result = RobotControlResult(success=True)
            goal_handle.succeed(result)
            return result
        elif goal_handle.request.task_type == RobotControlGoal.TASK_INSPECTION:
            result = RobotControlResult(success=True)
            goal_handle.succeed(result)
            return result
        else:
            goal_handle.abort()
            return None

def main(args=None):
    rclpy.init(args=args)
    action_server = RobotControlActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
