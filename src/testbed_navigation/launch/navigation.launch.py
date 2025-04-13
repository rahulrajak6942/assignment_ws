from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('testbed_navigation')

    # Include map server launch
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'map_loader.launch.py'])
        ])
    )

    # Include localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'localization.launch.py'])
        ])
    )

    # Create controller server node
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'controller_frequency': 20.0,
            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.5,
            'min_theta_velocity_threshold': 0.001,
            'failure_tolerance': 0.3,
            'progress_checker_plugin': 'nav2_controller::SimpleProgressChecker',
            'goal_checker_plugins': ['nav2_controller::SimpleGoalChecker'],
            'controller_plugins': ['nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'],
            'RegulatedPurePursuitController': {
                'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'desired_linear_vel': 0.5,
                'max_linear_accel': 2.5,
                'max_linear_decel': 2.5,
                'lookahead_dist': 0.6,
                'min_lookahead_dist': 0.3,
                'max_lookahead_dist': 0.9,
                'lookahead_time': 1.5,
                'rotate_to_heading_angular_vel': 1.8,
                'transform_tolerance': 0.1,
                'use_velocity_scaled_lookahead_dist': False,
                'min_approach_linear_velocity': 0.05,
                'approach_velocity_scaling_dist': 0.6
            }
        }]
    )

    # Create planner server node
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'planner_plugins': ['GridBased'],
            'GridBased.plugin': 'nav2_navfn_planner/NavfnPlanner',
            'GridBased.tolerance': 0.5,
            'GridBased.use_astar': False,
            'GridBased.allow_unknown': True,
            'GridBased.use_final_approach_orientation': True
        }]
    )

    # Create behavior tree node
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'transform_tolerance': 0.1,
            'default_server_timeout': 20.0,
            'default_nav_to_pose_bt_xml': str(PathJoinSubstitution([
                FindPackageShare('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery.xml'
            ])),
            'plugin_lib_names': [
                'nav2_compute_path_to_pose_action_bt_node',
                'nav2_compute_path_through_poses_action_bt_node',
                'nav2_follow_path_action_bt_node',
                'nav2_back_up_action_bt_node',
                'nav2_spin_action_bt_node',
                'nav2_wait_action_bt_node',
                'nav2_clear_costmap_service_bt_node',
                'nav2_is_stuck_condition_bt_node',
                'nav2_goal_reached_condition_bt_node',
                'nav2_goal_updated_condition_bt_node',
                'nav2_initial_pose_received_condition_bt_node',
                'nav2_reinitialize_global_localization_service_bt_node',
                'nav2_rate_controller_bt_node',
                'nav2_distance_controller_bt_node',
                'nav2_speed_controller_bt_node',
                'nav2_truncate_path_action_bt_node',
                'nav2_goal_updater_node_bt_node',
                'nav2_recovery_node_bt_node',
                'nav2_pipeline_sequence_bt_node',
                'nav2_round_robin_node_bt_node',
                'nav2_transform_available_condition_bt_node',
                'nav2_time_expired_condition_bt_node',
                'nav2_distance_traveled_condition_bt_node',
                'nav2_single_trigger_bt_node',
                'nav2_is_battery_low_condition_bt_node',
                'nav2_navigate_through_poses_action_bt_node',
                'nav2_navigate_to_pose_action_bt_node',
                'nav2_remove_passed_goals_action_bt_node'
            ]
        }]
    )

    # Create recovery node
    recovery = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'costmap_topic': '/local_costmap/costmap',
            'footprint_topic': '/local_costmap/published_footprint',
            'cycle_frequency': 10.0,
            'recovery_plugins': ['spin', 'backup', 'wait']
        }]
    )

    # Create waypoint follower node
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'loop_rate': 20,
            'stop_on_failure': False,
        }]
    )

    # Create global costmap node
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'update_frequency': 1.0,
            'publish_frequency': 1.0,
            'static_map': True,
            'rolling_window': False,
            'width': 10,
            'height': 10,
            'resolution': 0.05,
            'track_unknown_space': True,
            'always_send_full_costmap': True,
            'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
            'static_layer': {
                'plugin': 'nav2_costmap_2d::StaticLayer',
                'map_subscribe_transient_local': True
            },
            'obstacle_layer': {
                'plugin': 'nav2_costmap_2d::ObstacleLayer',
                'enabled': True,
                'observation_sources': 'scan',
                'scan': {
                    'topic': '/scan',
                    'max_obstacle_height': 2.0,
                    'clearing': True,
                    'marking': True,
                    'data_type': 'LaserScan',
                    'raytrace_max_range': 3.0,
                    'raytrace_min_range': 0.0,
                    'obstacle_max_range': 2.5,
                    'obstacle_min_range': 0.0
                }
            },
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'cost_scaling_factor': 3.0,
                'inflation_radius': 0.55
            }
        }]
    )

    # Create local costmap node
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'odom',
            'robot_base_frame': 'base_footprint',
            'update_frequency': 5.0,
            'publish_frequency': 2.0,
            'static_map': False,
            'rolling_window': True,
            'width': 3,
            'height': 3,
            'resolution': 0.05,
            'track_unknown_space': False,
            'always_send_full_costmap': True,
            'plugins': ['obstacle_layer', 'inflation_layer'],
            'obstacle_layer': {
                'plugin': 'nav2_costmap_2d::ObstacleLayer',
                'enabled': True,
                'observation_sources': 'scan',
                'scan': {
                    'topic': '/scan',
                    'max_obstacle_height': 2.0,
                    'clearing': True,
                    'marking': True,
                    'data_type': 'LaserScan',
                    'raytrace_max_range': 3.0,
                    'raytrace_min_range': 0.0,
                    'obstacle_max_range': 2.5,
                    'obstacle_min_range': 0.0
                }
            },
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'cost_scaling_factor': 3.0,
                'inflation_radius': 0.55
            }
        }]
    )

    # Create lifecycle manager for navigation
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server',
                'waypoint_follower',
                'global_costmap',
                'local_costmap'
            ]
        }]
    )

    return LaunchDescription([
        map_server_launch,
        localization_launch,
        controller_server,
        planner_server,
        bt_navigator,
        recovery,
        waypoint_follower,
        global_costmap,
        local_costmap,
        lifecycle_manager
    ])
