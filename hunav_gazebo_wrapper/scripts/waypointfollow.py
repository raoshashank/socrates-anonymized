#!/usr/bin/env python3

import sys
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from std_msgs.msg import Int32
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml,os,json
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
import rclpy.publisher
import rclpy.time

from visualization_msgs.msg import MarkerArray,Marker
class WaypointNavigator(BasicNavigator):
    def __init__(self, waypoints,scenario):
        super().__init__('waypoint_navigator')
        # Load waypoints from file
        self.robot_poses = waypoints
        self.get_logger().set_level(level=LoggingSeverity.WARN)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray,'/waypoint_markers',QoSProfile(depth=10))
        self.goal_pose_pub = self.create_publisher(PoseStamped,'/dummy_goal_pose',QoSProfile(depth=10))
        # self.status_pub = self.create_publisher(PoseStamped,'/finished_navigation',QoSProfile(depth=10))
        self.task_status_pub = self.create_publisher(Int32,'/task_status',QoSProfile(depth=10))
        self.status_pub = self.create_publisher(Int32,'/navigation_status',QoSProfile(depth=10))
        self.scenario = scenario
        # Subscribe to /amcl_pose to set the initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.robot_poses['initial_pose']['x']
        initial_pose.pose.position.y = self.robot_poses['initial_pose']['y']
        initial_quat = Rotation.from_euler('xyz',[0,0,self.robot_poses['initial_pose']['yaw']],degrees=False).as_quat()
        initial_pose.pose.orientation.x = initial_quat[0]
        initial_pose.pose.orientation.y = initial_quat[1]
        initial_pose.pose.orientation.z = initial_quat[2]
        initial_pose.pose.orientation.w = initial_quat[3]
        self.waypoints = []
        self.timer = self.create_timer(0.1,self.publishMarkers)
        self.setInitialPose(initial_pose)
        self.waitUntilNav2Active()
        self.navigate_waypoints()
    
    def publishMarkers(self):
        marray = MarkerArray()
        for i,pt in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id=i
            marker.type = 2
            marker.color.g = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.pose.position.x = pt.pose.position.x
            marker.pose.position.y = pt.pose.position.y
            marray.markers.append(marker)
        self.waypoint_marker_pub.publish(marray)

    def navigate_waypoints(self):
        waypoint_pose = PoseStamped()
        waypoint_pose.header.frame_id = 'map'
        waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        interaction_points = [v for k,v in self.scenario['interaction_points'].items()]
        print("Waypoints:")
        print(self.robot_poses['waypoints'])
        for i,pt in enumerate(self.robot_poses['waypoints']):
            #skip the interaction waypoint unless its the last node or the first node
            #self.get_logger().info(f'{i}: {self.scenario['trajectories']['robot'][i+1]}, IP:{interaction_points}')
            # if self.scenario['trajectories']['robot'][i+1] in interaction_points and \
            #     i!= len(self.robot_poses['waypoints'])-1 and i!= 0:
            #         continue  
            waypoint_pose.pose.position.x = pt['position']['x']
            waypoint_pose.pose.position.y = pt['position']['y']
            self.waypoints.append(deepcopy(waypoint_pose))
             
        # Commented out for safety; you can uncomment and use it
        self.followWaypoints(self.waypoints)

        # # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
        # # Simply print the current waypoint ID for the demonstration
        i = 0
        self.goal_pose_pub.publish(self.waypoints[-1]) #for hunav evaluator
        while not self.isTaskComplete():
            i = i + 1
            feedback = self.getFeedback()
            cw = Int32()
            cw.data = feedback.current_waypoint
            self.status_pub.publish(cw)
            if feedback and i % 5 == 0:
                self.get_logger().info('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(self.waypoints)))
                cw = Int32()
                cw.data = feedback.current_waypoint
                self.status_pub.publish(cw)
                
                stsmsg = Int32()
                stsmsg.data = -1
                self.task_status_pub.publish(stsmsg)
        
        if hasattr(self,"timer") and self.timer is not None:
            self.destroy_timer(self.timer)
            
        stsmsg = Int32()
        stsmsg.data = -1
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            stsmsg.data = TaskResult.SUCCEEDED.value
            self.get_logger().info('SUCCEEDED Waypoint following')
            
        elif result == TaskResult.CANCELED:
            stsmsg.data = TaskResult.CANCELED.value
            self.get_logger().warn('Waypoint following CANCELLED')
            
        elif result == TaskResult.FAILED:
            stsmsg.data = TaskResult.FAILED.value
            self.get_logger().error('Waypoint following FAILED')
            

        self.task_status_pub.publish(stsmsg)
        self.destroyNode()
        #rclpy.shutdown()
        return

def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    waypoint_file = os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'config','robot_poses.yaml')
    trajectory_file = os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),'config','trajectory.json')
    with open(waypoint_file,'r') as f:
        waypoints = yaml.safe_load(f)
    with open(trajectory_file,'r') as f:
        scenario = json.load(f)
    
    waypoint_navigator = WaypointNavigator(waypoints,scenario)
    #rclpy.spin(waypoint_navigator)
    try:
        waypoint_navigator.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
