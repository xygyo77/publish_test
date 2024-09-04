import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# パラメータを定義
pub_sub_type = DeclareLaunchArgument(
    'pub_sub_type',
    default_value='publisher',
    description='Type of the node (publisher or subscriber)'
)

publisher_count = DeclareLaunchArgument(
    'publisher_count',
    default_value='10',
    description='Number of publisher instances'
)

subscriber_count = DeclareLaunchArgument(
    'subscriber_count',
    default_value='10',
    description='Number of subscriber instances'
)

topic_count = DeclareLaunchArgument(
    'topic_count',
    default_value='10',
    description='Number of topics'
)

frequency = DeclareLaunchArgument(
    'frequency',
    default_value='10.0',
    description='Frequency of the publisher'
)

msg_size = DeclareLaunchArgument(
    'msg_size',
    default_value='100',
    description='Size of the message'
)

timer_period = DeclareLaunchArgument(
    'timer_period',
    default_value='100',
    description='Timer period'
)

qos_profile = DeclareLaunchArgument(
    'qos_profile',
    default_value='system',
    description='QoS profile'
)

# その他のパラメータも同様に定義します

# publisher_nodeの定義
publisher_node = Node(
    package='your_package_name',
    executable='publisher_node',
    name='publisher_node',
    output='both',
    parameters=[{'pub_sub_type': LaunchConfiguration('pub_sub_type'),
                 'publisher_count': LaunchConfiguration('publisher_count'),
                 'subscriber_count': LaunchConfiguration('subscriber_count'),
                 'topic_count': LaunchConfiguration('topic_count'),
                 'frequency': LaunchConfiguration('frequency'),
                 'msg_size': LaunchConfiguration('msg_size'),
                 'timer_period': LaunchConfiguration('timer_period'),
                 'qos_profile': LaunchConfiguration('qos_profile')}]  # パラメータを設定
)

# subscriber_nodeの定義
subscriber_node = Node(
    package='your_package_name',
    executable='subscriber_node',
    name='subscriber_node',
    output='both',
    parameters=[{'pub_sub_type': LaunchConfiguration('pub_sub_type'),
                 'publisher_count': LaunchConfiguration('publisher_count'),
                 'subscriber_count': LaunchConfiguration('subscriber_count'),
                 'topic_count': LaunchConfiguration('topic_count'),
                 'frequency': LaunchConfiguration('frequency'),
                 'msg_size': LaunchConfiguration('msg_size'),
                 'timer_period': LaunchConfiguration('timer_period'),
                 'qos_profile': LaunchConfiguration('qos_profile')}]  # パラメータを設定
)

# LaunchDescriptionの作成
ld = launch.LaunchDescription([
    pub_sub_type,
    publisher_count,
    subscriber_count,
    topic_count,
    frequency,
    msg_size,
    timer_period,
    qos_profile,
    # その他のパラメータも同様に追加します
    publisher_node,
    subscriber_node
])

# LaunchDescriptionの保存
with open('publisher_and_subscriber.launch.py', 'w') as file:
    file.write(ld.format())