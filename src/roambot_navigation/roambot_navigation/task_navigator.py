import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class TaskNavigator(Node):
    def __init__(self):
        super().__init__('task_navigator')

        self.declare_parameter('robot', '')
        self.declare_parameter('task', '')
        self.declare_parameter('task_file', '')

        self.robot = self.get_parameter('robot').value
        self.task_name = self.get_parameter('task').value
        task_file = self.get_parameter('task_file').value

        if not self.robot or not self.task_name:
            raise ValueError(
                'Provide robot:=scout|service and a task:=<task_name>.'
            )

        if not task_file:
            package_share = get_package_share_directory(
                'roambot_navigation'
            )
            task_file = str(
                Path(package_share)
                / 'config'
                / 'warehouse_task_poses.yaml'
            )

        with open(task_file, 'r', encoding='utf-8') as stream:
            task_data = yaml.safe_load(stream)

        work_tasks = task_data.get('work_tasks', {})
        if self.task_name not in work_tasks:
            available = ', '.join(sorted(work_tasks))
            raise ValueError(
                f'Unknown work task "{self.task_name}". '
                f'Available: {available}'
            )

        self.task = work_tasks[self.task_name]
        allowed_robots = self.task.get('allowed_robots', [])

        if self.robot not in allowed_robots:
            raise ValueError(
                f'Task "{self.task_name}" is not assigned to '
                f'robot "{self.robot}".'
            )

        action_name = f'/{self.robot}/navigate_to_pose'
        self.client = ActionClient(self, NavigateToPose, action_name)

    def create_goal(self):
        x, y = self.task['pose']

        if 'face_point' in self.task:
            face_x, face_y = self.task['face_point']
            yaw = math.atan2(face_y - y, face_x - x)
        else:
            yaw = self.task.get('yaw', 0.0)

        self.get_logger().info(f'Goal position=({x:.2f}, {y:.2f}), final yaw={yaw:.3f} rad')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'

        # A zero timestamp means “use the latest available transform”.
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        return goal

    def navigate(self):
        self.get_logger().info(
            f'Sending {self.robot} to task: {self.task_name}'
        )

        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f'Navigation action server unavailable for {self.robot}.'
            )
            return 1

        goal_future = self.client.send_goal_async(self.create_goal())
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected.')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Task completed: {self.task_name}'
            )
            return 0

        self.get_logger().error(
            f'Task failed: {self.task_name}, '
            f'Nav2 status={result.status}'
        )
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TaskNavigator()
        exit_code = node.navigate()
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'task_navigator error: {error}')
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    main()
