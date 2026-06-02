import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import rmf_adapter
import rmf_adapter.vehicletraits as vehicletraits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import yaml
import time
import threading
import math
import gc
import os
import psutil
from datetime import datetime, timedelta

class Nav2RobotCommandHandle(rmf_adapter.RobotCommandHandle):
    def __init__(self, node, robot_name, fleet_name):
        rmf_adapter.RobotCommandHandle.__init__(self)
        self.node = node
        self.robot_name = robot_name
        self.fleet_name = fleet_name
        self.action_client = ActionClient(node, NavigateToPose, f'/{robot_name}/navigate_to_pose')
        self.update_handle = None
        self.current_goal_handle = None
        
        # Threading and control variables
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._goal_completed_event = threading.Event()
        self._goal_status = None

    def follow_new_path(self, waypoints, estimate_arrival_cb, arrival_cb):
        self.node.get_logger().info(f"[{self.robot_name}] New path received with {len(waypoints)} waypoints")
        self.interrupt()
        gc.collect()
        
        self._quit_path_event.clear()
        self._goal_completed_event.clear()
        
        def _follow_path():
            self.node.get_logger().info(f"[{self.robot_name}] Starting path execution...")
            for i, wp in enumerate(waypoints):
                if self._quit_path_event.is_set():
                    break
                
                # Transform RMF coordinates -> ROS 2 (reflected Y)
                x_ros = wp.position[0] - 10.0
                y_ros = -10.0 - wp.position[1]
                yaw_ros = -wp.position[2]
                
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = "map"
                goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = x_ros
                goal_msg.pose.pose.position.y = y_ros
                
                # Convert yaw_ros to quaternion
                goal_msg.pose.pose.orientation.z = math.sin(yaw_ros / 2.0)
                goal_msg.pose.pose.orientation.w = math.cos(yaw_ros / 2.0)
                
                self.node.get_logger().info(
                    f"[{self.robot_name}] Navigating to waypoint {i+1}/{len(waypoints)}: "
                    f"RMF[{wp.position[0]:.2f}, {wp.position[1]:.2f}] -> ROS[{x_ros:.2f}, {y_ros:.2f}]"
                )
                
                self._goal_completed_event.clear()
                self._goal_status = None
                
                self.action_client.wait_for_server()
                
                # Feedback callback to update RMF with estimated arrival time
                def feedback_callback(feedback_msg):
                    try:
                        fb = feedback_msg.feedback
                        sec = fb.estimated_time_remaining.sec
                        nanosec = fb.estimated_time_remaining.nanosec
                        seconds_remaining = sec + nanosec * 1e-9
                        if seconds_remaining <= 0.0:
                            seconds_remaining = fb.distance_remaining / 0.3 # fallback average speed
                        estimate_arrival_cb(i, timedelta(seconds=seconds_remaining))
                    except Exception as e:
                        pass

                send_goal_future = self.action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=feedback_callback
                )
                
                while not send_goal_future.done():
                    if self._quit_path_event.is_set():
                        break
                    time.sleep(0.05)
                
                if self._quit_path_event.is_set():
                    break
                    
                goal_handle = send_goal_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    self.node.get_logger().error(f"[{self.robot_name}] Nav2 goal rejected at waypoint {i+1}!")
                    break
                    
                self.current_goal_handle = goal_handle
                get_result_future = goal_handle.get_result_async()
                
                def result_callback(result_future):
                    from action_msgs.msg import GoalStatus
                    res = result_future.result()
                    if res is not None:
                        self._goal_status = res.status
                    else:
                        self._goal_status = GoalStatus.STATUS_UNKNOWN
                    self._goal_completed_event.set()
                    
                get_result_future.add_done_callback(result_callback)
                
                while not self._goal_completed_event.is_set():
                    if self._quit_path_event.is_set():
                        self.node.get_logger().info(f"[{self.robot_name}] Canceling active Nav2 goal...")
                        goal_handle.cancel_goal_async()
                        break
                    time.sleep(0.05)
                    
                if self._quit_path_event.is_set():
                    break
                    
                from action_msgs.msg import GoalStatus
                if self._goal_status == GoalStatus.STATUS_SUCCEEDED:
                    self.node.get_logger().info(f"[{self.robot_name}] Reached waypoint {i+1}/{len(waypoints)}")
                else:
                    self.node.get_logger().warn(f"[{self.robot_name}] Failed at waypoint {i+1} with status: {self._goal_status}")
                    break
            
            # Finished path
            self.current_goal_handle = None
            from action_msgs.msg import GoalStatus
            if not self._quit_path_event.is_set() and self._goal_status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info(f"[{self.robot_name}] Path navigation complete.")
                arrival_cb()
            else:
                self.node.get_logger().info(f"[{self.robot_name}] Path navigation aborted or failed.")
            gc.collect()
                
        self._follow_path_thread = threading.Thread(target=_follow_path)
        self._follow_path_thread.start()

    def interrupt(self):
        self._quit_path_event.set()
        self._goal_completed_event.set()
        
        if self.current_goal_handle is not None:
            try:
                self.node.get_logger().info(f"[{self.robot_name}] Canceling active goal in interrupt...")
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.node.get_logger().warn(f"[{self.robot_name}] Error canceling goal in interrupt: {e}")
            self.current_goal_handle = None
            
        if self._follow_path_thread is not None:
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
        gc.collect()

    def stop(self):
        self.node.get_logger().info(f"[{self.robot_name}] Stop requested")
        self.interrupt()

class Nav2FleetAdapter:
    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Initializing Nav2 Fleet Adapter...")
        
        import os
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('tb3_rmf')
        config_dir = os.path.join(share_dir, 'config')
        with open(os.path.join(config_dir, 'fleet_config.yaml'), 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Coordinate transformation offsets (from map.yaml origin)
        self.x_offset = -10.0
        self.y_offset = -10.0
        # RMF's building_map_generator uses y-down, so we need to account for image height if we want perfect match,
        # but since we generated the graph with a scale, we can just use a simple offset if it's consistent.
        # Based on AMCL vs 0.yaml, x_ros = x_rmf - 10.0 seems correct.
        # For Y: y_rmf is -9.7, y_ros is -0.5. y_ros = y_rmf + 9.2? 
        # Actually, let's use the full height to be safe if we knew it.
        # But let's try a simple offset first.
            
        # Initialize RMF Adapter with wait time to avoid None return
        self.node.get_logger().info("Connecting to RMF Traffic Schedule (waiting up to 30s)...")
        self.adapter = rmf_adapter.Adapter.make('rmf_fleet_adapter_core', wait_time=timedelta(seconds=30))
        if self.adapter is None:
            raise RuntimeError("Failed to connect to RMF Traffic Schedule!")
        self.node.get_logger().info("Successfully connected to RMF Traffic Schedule.")
            
        # Profile and Traits
        footprint = rmf_adapter.geometry.make_final_convex_circle(0.3)
        self.profile = rmf_adapter.vehicletraits.Profile(footprint)
        self.traits = rmf_adapter.vehicletraits.VehicleTraits(
            linear=rmf_adapter.vehicletraits.Limits(0.5, 0.2),
            angular=rmf_adapter.vehicletraits.Limits(0.5, 0.2),
            profile=self.profile
        )

        # Graph
        nav_graph = rmf_adapter.graph.parse_graph(os.path.join(config_dir, '0.yaml'), self.traits)

        self.fleet = self.adapter.add_fleet(
            self.config['rmf_fleet_adapter']['fleet_name'], 
            self.traits,
            nav_graph
        )
        self.nav_graph = nav_graph
        
        # Configure task planner params so the fleet can bid for tasks
        # These are basic parameters for battery and power consumption
        import rmf_adapter.battery as battery
        battery_system = battery.BatterySystem.make(24.0, 40.0, 10.0)
        mechanical_system = battery.MechanicalSystem.make(20.0, 10.0, 0.7)
        power_system = battery.PowerSystem.make(1.0)
        
        motion_sink = battery.SimpleMotionPowerSink(battery_system, mechanical_system)
        ambient_sink = battery.SimpleDevicePowerSink(battery_system, power_system)
        tool_sink = battery.SimpleDevicePowerSink(battery_system, battery.PowerSystem.make(0.0))

        self.fleet.set_task_planner_params(
            battery_system,
            motion_sink,
            ambient_sink,
            tool_sink,
            0.2, # recharge_threshold
            1.0, # recharge_soc
            True # account_for_battery_drain
        )

        self.robots = {}
        self.last_update_time = {}
        self.last_pose = {} # Store last pose to calculate delta
        for robot_name in self.config['rmf_fleet_adapter']['robots']:
            self.add_robot(robot_name)

        # Start persistent resource monitor
        def resource_monitor(node, robots):
            log_path = '/tmp/resource_monitor.log'
            node.get_logger().info(f"Resource monitor starting, logging to {log_path}")
            process = psutil.Process(os.getpid())
            
            with open(log_path, 'w') as f:
                f.write("timestamp,rss_mb,vms_mb,system_ram_pct,threads_count,robot_states\n")
                f.flush()
                os.fsync(f.fileno())
                
                while rclpy.ok():
                    try:
                        mem_info = process.memory_info()
                        rss = mem_info.rss / (1024 * 1024)
                        vms = mem_info.vms / (1024 * 1024)
                        system_ram_pct = psutil.virtual_memory().percent
                        threads = len(process.threads())
                        
                        states = []
                        for name, cmd in robots.items():
                            active_goal = "none" if cmd.current_goal_handle is None else "active"
                            thread_alive = "dead" if cmd._follow_path_thread is None else ("alive" if cmd._follow_path_thread.is_alive() else "finished")
                            states.append(f"{name}[goal:{active_goal},thread:{thread_alive}]")
                        states_str = ";".join(states)
                        
                        log_line = f"{time.time()},{rss:.2f},{vms:.2f},{system_ram_pct:.2f},{threads},{states_str}\n"
                        f.write(log_line)
                        f.flush()
                        os.fsync(f.fileno())
                    except Exception as e:
                        pass
                    time.sleep(1.0)
                    
        self.monitor_thread = threading.Thread(
            target=resource_monitor, 
            args=(self.node, self.robots), 
            daemon=True
        )
        self.monitor_thread.start()

    def add_robot(self, robot_name):
        self.node.get_logger().info(f"Adding robot {robot_name} to fleet")
        
        # Get start waypoint index from name
        start_wp_name = self.config['rmf_fleet_adapter']['robots'][robot_name]['rmf_config']['start_waypoint']
        start_wp_idx = None
        for i in range(self.nav_graph.num_waypoints):
            wp = self.nav_graph.get_waypoint(i)
            if wp.waypoint_name == start_wp_name:
                start_wp_idx = i
                break
        
        if start_wp_idx is None:
            self.node.get_logger().error(f"Could not find waypoint {start_wp_name} in graph!")
            start_wp_idx = 0

        cmd_handle = Nav2RobotCommandHandle(self.node, robot_name, self.config['rmf_fleet_adapter']['fleet_name'])
        
        # Sync with RMF schedule node timeline using adapter.now()
        dt = self.adapter.now()
        
        start = rmf_adapter.plan.Start(dt, start_wp_idx, 0.0)
        
        def handle_cb(update_handle):
            self.node.get_logger().info(f"Received update handle for {robot_name}")
            cmd_handle.update_handle = update_handle

        self.fleet.add_robot(cmd_handle, robot_name, self.profile, [start], handle_cb)
        self.robots[robot_name] = cmd_handle
        self.last_update_time[robot_name] = 0.0
        self.last_pose[robot_name] = (self.x_offset + 10.0, - (self.y_offset + 10.0), 0.0) # Approx start

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.node.create_subscription(
            PoseWithCovarianceStamped, 
            f'/{robot_name}/amcl_pose', 
            lambda msg, name=robot_name: self.pose_callback(name, msg), 
            qos_profile
        )

    def pose_callback(self, robot_name, msg):
        handle = self.robots.get(robot_name)
        if handle and handle.update_handle:
            now_seconds = self.node.get_clock().now().nanoseconds / 1e9
            
            # 1. Time throttle (1Hz is plenty for RMF)
            if now_seconds - self.last_update_time.get(robot_name, 0.0) < 1.0:
                return
            
            # Always update the throttle timestamp immediately to prevent bypass when stationary
            self.last_update_time[robot_name] = now_seconds
            
            # 2. Movement threshold
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            curr_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            
            last_x, last_y, last_yaw = self.last_pose.get(robot_name, (0.0, 0.0, 0.0))
            dist = math.sqrt((curr_x - last_x)**2 + (curr_y - last_y)**2)
            yaw_diff = abs(curr_yaw - last_yaw)
            
            if dist < 0.05 and yaw_diff < 0.05: # Thresholds: 5cm or 0.05rad
                return

            self.last_pose[robot_name] = (curr_x, curr_y, curr_yaw)

            try:
                # Transform ROS 2 -> RMF
                x_rmf = msg.pose.pose.position.x + 10.0
                y_rmf = -(msg.pose.pose.position.y + 10.0)
                
                # Convert quaternion to yaw
                q = msg.pose.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # Use RMF's native timezone/timeline synchronized time
                dt = self.adapter.now()
                
                # Find nearest waypoint
                nearest_wp_idx = 0
                min_dist = float('inf')
                for i in range(self.nav_graph.num_waypoints):
                    wp = self.nav_graph.get_waypoint(i)
                    dist = math.sqrt((wp.location[0] - x_rmf)**2 + (wp.location[1] - y_rmf)**2)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_wp_idx = i
                
                start = rmf_adapter.plan.Start(dt, nearest_wp_idx, -yaw, [x_rmf, y_rmf])
                # Update RMF position - REQUIRED TO BE A LIST in Humble
                handle.update_handle.update_position([start])
            except Exception as e:
                self.node.get_logger().error(f"Error in pose_callback for {robot_name}: {e}")

def main(args=None):
    try:
        rclpy.init(args=args)
    except:
        pass # Already initialized
        
    try:
        rmf_adapter.init_rclcpp()
        
        # Create a standard ROS node for subscriptions/actions
        helper_node = Node('nav2_fleet_adapter_helper')
        helper_node.get_logger().info("Starting Nav2 Fleet Adapter Helper Node...")
        
        adapter_manager = None
        try:
            adapter_manager = Nav2FleetAdapter(helper_node)
        except Exception as e:
            helper_node.get_logger().error(f"Failed to initialize Nav2FleetAdapter: {e}")
            helper_node.destroy_node()
            return

        executor = MultiThreadedExecutor()
        executor.add_node(helper_node)
        
        # Start RMF adapter thread
        helper_node.get_logger().info("Starting RMF Adapter thread...")
        adapter_thread = threading.Thread(target=lambda: adapter_manager.adapter.start(), daemon=True)
        adapter_thread.start()
        
        helper_node.get_logger().info("Fleet Adapter is now spinning.")
        executor.spin()
    except Exception as e:
        print(f"Critical Error in Nav2 Fleet Adapter: {e}")
    finally:
        if 'helper_node' in locals():
            helper_node.get_logger().info("Shutting down Nav2 Fleet Adapter...")
            helper_node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
