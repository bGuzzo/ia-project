import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory("plansys2_project")
    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace"
    )

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("plansys2_bringup"),
                "launch",
                "plansys2_bringup_launch_monolithic.py",
            )
        ),
        launch_arguments={
            "model_file": example_dir + "/pddl/rescue_domain_plansys.pddl",
            "namespace": namespace,
        }.items(),
    )
    
    # Specify the actions
    drop_box_of_carrier_cmd = Node(
        package='plansys2_project',
        executable='drop_box_of_carrier_node',
        name='drop_box_of_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    drop_content_of_box_cmd = Node(
        package='plansys2_project',
        executable='drop_content_of_box_node',
        name='drop_content_of_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_robot_with_carrier_cmd = Node(
        package='plansys2_project',
        executable='move_robot_with_carrier_node',
        name='move_robot_with_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_robot_cmd = Node(
        package='plansys2_project',
        executable='move_robot_node',
        name='move_robot_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    put_box_on_carrier_cmd = Node(
        package='plansys2_project',
        executable='put_box_on_carrier_node',
        name='put_box_on_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    put_content_in_box_cmd = Node(
        package='plansys2_project',
        executable='put_content_in_box_node',
        name='put_content_in_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)
    ld.add_action(drop_box_of_carrier_cmd)
    ld.add_action(drop_content_of_box_cmd)
    ld.add_action(move_robot_with_carrier_cmd)
    ld.add_action(move_robot_cmd)
    ld.add_action(put_box_on_carrier_cmd)
    ld.add_action(put_content_in_box_cmd)
    
    return ld
    
    
    



