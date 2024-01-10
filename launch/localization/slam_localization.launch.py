from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
          parameters=[
            get_package_share_directory("f1tenth_launch") + '/config/localization/localizer_slam.yaml'
          ],
          package='slam_toolbox',
          executable='localization_slam_toolbox_node',
          name='slam_toolbox',
          output='screen'
        )
    ])