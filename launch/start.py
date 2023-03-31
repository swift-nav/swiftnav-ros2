'''
Copyright (C) 2015-2023 Swift Navigation Inc.
Contact: https://support.swiftnav.com

This source is subject to the license found in the file 'LICENSE' which must
be be distributed together with this source. All other rights reserved.

THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  launch_desc = LaunchDescription()
  config = os.path.join(get_package_share_directory('swiftnav_ros2_driver'),
                        'config',
                        'settings.yaml'
                        )
  node = Node(
      package='swiftnav_ros2_driver',
      executable='sbp-to-ros',
      parameters=[config]
  )

  launch_desc.add_action(node)
  return launch_desc
