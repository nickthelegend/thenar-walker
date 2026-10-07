from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    share = Path(get_package_share_directory('so101_description'))
    description = xacro.process_file(str(share / 'urdf/so101.urdf.xacro')).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/so101.rviz')]),
    ])
