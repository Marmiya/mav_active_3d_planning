#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import SetBool
import math

class Trajectory2Pose(object):
    def __init__(self):
        self.ns_planner = rospy.get_param('~ns_planner', "/firefly/planner_node")
        self.pub = rospy.Publisher('/command/pose', Pose, queue_size=10)
        self.sub = rospy.Subscriber('/command/trajectory', MultiDOFJointTrajectory, self.callback, queue_size=1)
        self.gt_pose_sub = rospy.Subscriber('ground_truth/pose', PoseStamped, self.gt_callback, queue_size=1)

        self.current_pose = None  # Current position of the drone
        self.target_queue = []  # Queue of target points
        self.distance_threshold = 0.5  # Distance threshold to determine if target is reached

        rospy.wait_for_service(self.ns_planner + "/toggle_running")
        try:
            run_planner_srv = rospy.ServiceProxy(self.ns_planner + "/toggle_running", SetBool)
            response = run_planner_srv(True)

            if response.success:
                rospy.loginfo("Service call succeeded: Planner started.")
            else:
                rospy.logerr("Service call failed: Planner could not be started.")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def callback(self, data):
        # When receiving a new trajectory, store all points in the queue
        self.target_queue = []  # Clear the queue, load new trajectory points
        for point in data.points:
            pose = Pose()
            pose.position.x = point.transforms[0].translation.x
            pose.position.y = point.transforms[0].translation.y
            pose.position.z = point.transforms[0].translation.z
            pose.orientation = point.transforms[0].rotation
            self.target_queue.append(pose)
        
        # only keep the last point
        self.target_queue = self.target_queue[-1:]
        
        # Check if two points are close to each other and remove the redundant one
        # we should check both position and orientation
        # i = 0
        # while i < len(self.target_queue) - 1:
        #     if self.calculate_distance(self.target_queue[i], self.target_queue[i+1]) < 0.1 \
        #         and self.calculate_yaw(self.target_queue[i].orientation) - \
        #             self.calculate_yaw(self.target_queue[i+1].orientation) < 0.2:
        #         self.target_queue.pop(i)
        #     else:
        #         i += 1

        # check if the target is empty
        if not self.target_queue:
            return
        
        # Publish the first target point
        if self.target_queue:
            self.publish_next_target()

    def publish_next_target(self):
        """Publish the next target point from the queue"""
        if self.target_queue:
            next_target = self.target_queue[0]  # Get the next target point (first point in the queue)
            self.pub.publish(next_target)
            rospy.loginfo("Publishing next position: {}".format(next_target.position))
            rospy.loginfo("Publishing next yaw: {}".format(self.calculate_yaw(next_target.orientation)))
        else:
            rospy.loginfo("All targets have been reached.")

    def gt_callback(self, pose):
        # Update the current position of the drone
        self.current_pose = pose.pose
        
        # check if the target is empty
        if not self.target_queue:
            return

        # Check if the drone has reached the current target
        if self.current_pose and self.target_queue:
            current_target = self.target_queue[0]  # Get the current target (first point in the queue)
            distance = self.calculate_distance(self.current_pose, current_target)
            if distance < self.distance_threshold:
                rospy.loginfo("Reached target position.")
                self.target_queue.pop(0)  # Remove the reached target
                self.publish_next_target()  # Publish the next target

    def calculate_distance(self, pose1, pose2):
        """Calculate the Euclidean distance between two poses"""
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2 +
            (pose1.position.z - pose2.position.z) ** 2
        )
        
    def calculate_yaw(self, orientation):
        """Calculate the yaw angle from a quaternion orientation"""
        return math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        )


if __name__ == '__main__':
    rospy.init_node('publish_poses_node')
    t2p = Trajectory2Pose()
    rospy.spin()
