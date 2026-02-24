from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # loadcell_controller 패키지의 share 디렉토리 경로 획득
    pkg_share = get_package_share_directory('loadcell_controller')

    # 파라미터 파일 전체 경로 생성
    param_file = os.path.join(pkg_share, 'config', 'loadcell_controller_1.yaml')

    return LaunchDescription([
        Node(
            package='loadcell_controller',
            executable='loadcell_controller_node',
            name='loadcell_controller_node_1',
            output='screen',
            parameters=[param_file],
        )
    ])
