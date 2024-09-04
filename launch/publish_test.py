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

# その他のパラメータも同様に定義します

# publisher_nodeの定義
publisher_node = Node(
    package='publish_test',
    executable='publisher_node',
    name='publisher_node',
    output='both',
    parameters=[LaunchConfiguration('pub_sub_type'),
                LaunchConfiguration('topic_count'),
                LaunchConfiguration('frequency'),
                LaunchConfiguration('msg_size'),
                LaunchConfiguration('qos_profile')]
)

# subscriber_nodeの定義
subscriber_node = Node(
    package='your_package_name',
    executable='subscriber_node',
    name='subscriber_node',
    output='both',
    parameters=[LaunchConfiguration('pub_sub_type'),
                LaunchConfiguration('topic_count'),
                LaunchConfiguration('qos_profile')]
)

# 条件分岐によるNode起動
ld = launch.LaunchDescription([
    pub_sub_type,
    # その他のパラメータ定義

    IfCondition(
        LaunchConfiguration('pub_sub_type') == "publisher",
        publisher_node
    ),
    IfCondition(
        LaunchConfiguration('pub_sub_type') == "subscriber",
        subscriber_node
    ),
])

# LaunchDescriptionの保存
with open('publisher_and_subscriber.launch.py', 'w') as file:
    file.write(ld.format())