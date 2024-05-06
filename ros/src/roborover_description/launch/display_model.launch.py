import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'roborover_description'

  pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
  robot_description_path = os.path.join(pkg_share, 'urdf/roborover.urdf')
  rviz_config_path = os.path.join(pkg_share, 'rviz/roborover_model.rviz')
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  with open(robot_description_path, 'r') as infp:
      robot_desc = infp.read()
  rsp_params = {'robot_description': robot_desc}

  robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[rsp_params, {'use_sim_time': use_sim_time}]
  )
  joint_state_publisher_node = launch_ros.actions.Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher'
  )
  joint_state_publisher_gui_node = launch_ros.actions.Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui'
  )
  rviz_node = launch_ros.actions.Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_path],
  )

  return launch.LaunchDescription([
    joint_state_publisher_node,
    joint_state_publisher_gui_node,
    robot_state_publisher_node,
    rviz_node
  ])
