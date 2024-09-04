import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    ld = launch.LaunchDescription()

    # parameters definition
    pub_sub_type = DeclareLaunchArgument(
        'pub_sub_type',
        default_value='publisher',
        description='Type of the node (publisher or subscriber)'
    )
    topic_count = DeclareLaunchArgument(
        'topic_count',
        default_value='5',
        description='Count of the topic (integer > 0)'
    )
    frequency = DeclareLaunchArgument(
        'frequency',
        default_value='10',
        description='Frequency of the topic (Hz > 0)'
    )
    msg_size = DeclareLaunchArgument(
        'msg_size',
        default_value='10',
        description='Msg_size of the topic (Integer > 0)'
    )

    # publisher_node
    publisher_node = Node(
        package='publish_test',
        executable='publisher_node',
        name='publisher_node',
        output='both',
        parameters=[LaunchConfiguration('pub_sub_type'),
                    LaunchConfiguration('topic_count'),
                    LaunchConfiguration('frequency'),
                    LaunchConfiguration('msg_size')]
    )

    # subscriber_node
    subscriber_node = Node(
        package='publish_test',
        executable='subscriber_node',
        name='subscriber_node',
        output='both',
        parameters=[LaunchConfiguration('pub_sub_type'),
                    LaunchConfiguration('topic_count')]
    )

    pub_sub_type = 'subscriber'
    ld.add_action(subscriber_node)
    pub_sub_type = 'publisher'
    ld.add_action(publisher_node)

    return ld