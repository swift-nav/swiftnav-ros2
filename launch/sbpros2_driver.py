import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  launch_desc = LaunchDescription()
  config = os.path.join(get_package_share_directory('swiftnav_ros2_driver'),
                        'config',
                        'params.yaml'
                        )
  node = Node(
      package='swiftnav_ros2_driver',
      executable='sbp-to-ros',
      parameters=[config]
  )

  launch_desc.add_action(node)
  return launch_desc
