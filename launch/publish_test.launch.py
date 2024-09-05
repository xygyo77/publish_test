import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression

def generate_launch_description():

    ld = launch.LaunchDescription()

    # Parameters definition
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
        description='Message size of the topic (integer > 0)'
    )
    output_suppressed = DeclareLaunchArgument(
        'output_suppressed',
        default_value='false',
        description='Boolean flag to suppress output (true/false)'
    )

    # Add launch arguments to the launch description
    ld.add_action(topic_count)
    ld.add_action(frequency)
    ld.add_action(msg_size)
    ld.add_action(output_suppressed)

    # Publisher node
    publisher_node = Node(
        package='publish_test',
        executable='publisher_node',
        name='publisher_node',
        output='both',
        parameters=[{
            'topic_count': LaunchConfiguration('topic_count'),
            'frequency': LaunchConfiguration('frequency'),
            'msg_size': LaunchConfiguration('msg_size'),
            'output_suppressed': LaunchConfiguration('output_suppressed')
        }]
    )

    # Subscriber node
    subscriber_node = Node(
        package='publish_test',
        executable='subscriber_node',
        name='subscriber_node',
        output='both',
        parameters=[{
            'topic_count': LaunchConfiguration('topic_count'),
            'output_suppressed': LaunchConfiguration('output_suppressed')
        }]
    )

    # Add nodes to the launch description
    ld.add_action(subscriber_node)
    ld.add_action(publisher_node)

    return ld
