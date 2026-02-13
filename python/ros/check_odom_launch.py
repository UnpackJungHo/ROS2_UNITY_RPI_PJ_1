from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부 (기본값 True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Unity) clock if true'),

        # RViz2 실행 (config 파일 없이 기본 실행, 필요하면 -d 옵션 추가 가능)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', ''] # 빈 설정으로 시작하거나, 저장된 설정 파일 경로를 넣으세요
        )
    ])
