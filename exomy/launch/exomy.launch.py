import os
import yaml
from tempfile import NamedTemporaryFile
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

namespace_ = 'exomy'

def launch_setup(context, *args, **kwargs):
    # Get the path to the exomy.yaml parameter file
    exomy_config = os.path.join(get_package_share_directory('exomy'),'exomy.yaml')
    namespace = LaunchConfiguration('namespace').perform(context)

    configured_params = prepend_namespace_to_yaml(exomy_config, namespace)

    robot = Node(
        package='exomy',
        executable='robot_node',
        name='robot_node',
        namespace=namespace_,
        parameters=[configured_params],
        output='screen'
    )
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace_,
        output='screen'
    )
    joystick = Node(
        package='exomy',
        executable='joystick_parser_node',
        name='joystick_parser_node',
        namespace=namespace_,
        output='screen'
    )
    motors = Node(
        package='exomy',
        executable='motor_node',
        name='motor_node',
        namespace=namespace_,
        parameters=[configured_params],
        output='screen'
    )

    return [robot, joy, joystick, motors]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='exomy', description='Top-level namespace'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        OpaqueFunction(function=launch_setup)
    ])


def prepend_namespace_to_yaml(input_file, namespace):
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    namespaced_data = {}
    for node_name, node_config in data.items():
        namespaced_key = f'/{namespace}/{node_name}'
        namespaced_data[namespaced_key] = node_config

    tmp_file = NamedTemporaryFile(delete=False, mode='w', suffix='.yaml')
    yaml.dump(namespaced_data, tmp_file)
    tmp_file.close()

   # with open(tmp_file.name, 'r') as f:
   #     print(f.read())  # print temp file content

    return tmp_file.name
