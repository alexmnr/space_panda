from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):
    ip = context.launch_configurations['ip']
    log_level = context.launch_configurations['log_level']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']

    print("")
    print("Starting driver with paramaters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" ip:                  " + ip)
    print("")

    pkg_name = "space_panda_bringup"
    ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "config", "ros2_controllers.yaml"]
    )
    description_file = PathJoinSubstitution([FindPackageShare("space_panda_description"), "urdf", "space_panda.urdf.xacro"])
    robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                description_file,
                " ",
                "robot_ip:=",
                ip,
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "use_mock_hardware:=false",
                ])
    robot_description = {
            "robot_description": ParameterValue(robot_description_content, value_type=str)
            }

    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[
            robot_description,
            {'publish_frequency': 1000.0}
            ]
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            ParameterFile(ros2_controllers_file, allow_substs=True),
            robot_description,
            ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "space_panda_controller",
            "--ros-args", "--log-level", log_level,
            ]
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_state_broadcaster",
            "--ros-args", "--log-level", log_level,
            ]
        ))

    return nodes

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
            DeclareLaunchArgument(
                'ns',
                default_value='',
                description='namespace of the robot (used as prefix, so needed if running multiple robots)'
                )
            )
    declared_arguments.append(
            SetLaunchConfiguration('tf_prefix', PythonExpression(["'", LaunchConfiguration('ns'), "' + '_' if '", LaunchConfiguration('ns'), "' else ''"]))
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "ip", 
                default_value="192.168.19.151",
                description="IP address by which the robot can be reached."
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                'log_level',
                default_value='error',
                description="Log Level to use for all nodes",
                choices=["info", "debug", "error"],
                )
            )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

