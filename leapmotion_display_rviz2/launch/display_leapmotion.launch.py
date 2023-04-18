import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    Command
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Set LOG format
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

# Camera model (force value)
camera_model = 'leapmotion'


def launch_setup(context, *args, **kwargs):
    urdf_tutorial_path = get_package_share_path('leapmotion_display_rviz2')
    default_model_path = urdf_tutorial_path / 'urdf/robot.urdf'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'robot.urdf'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')

    print('urdf_file_name : {}'.format(urdf_file_name))
    urdf = os.path.join(
        get_package_share_directory('leapmotion_display_rviz2'),
        'urdf',
        'robot' + '.urdf'
    )

    print("urdf: ", urdf)

    start_zed_node = LaunchConfiguration('start_leapmotion_node')
    camera_name = LaunchConfiguration('camera_name')

    camera_name_val = camera_name.perform(context)

    if (camera_name_val == ""):
        camera_name_val = camera_model


    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    print("robot_description: ", robot_description)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )    

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('leapmotion_display_rviz2'),
        'rviz2',
        camera_name_val + '.rviz'
    )

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name,
        executable='rviz2',
        name=camera_name_val+'_rviz2',
        output='screen',
        arguments=[["-d"], [config_rviz2]],
    )

    urdf_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf])

    # Leap Motion Wrapper launch file
    leapmotion_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('leapmotion_wrapper'),
            '/launch/' + camera_model + '.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    return [
        model_arg,
        rviz2_node,
        urdf_node,
        leapmotion_wrapper_launch,
        robot_state_publisher_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_leapmotion_node',
                default_value='True',
                description='Set to `False` to start only Rviz2 if a Leap Motion node is already running.'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text=""),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`. Leave empty to use the camera model as camera name.'),
                OpaqueFunction(function=launch_setup)
        ]
    )
