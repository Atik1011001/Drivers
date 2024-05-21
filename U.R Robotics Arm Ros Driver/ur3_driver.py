#!/usr/bin/env python3
# Python 2/3 compatibility imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
    return True 


class UR3MOVE(object):

    def __init__(self, group_name=""):
        """
            Initialize the UR3 Move Group.

            This constructor sets up the necessary components for interacting with the UR3 robot
            using MoveIt! motion planning. It initializes the move group with the given group_name,
            sets the planning time to 10 seconds, and sets up the planning frame and end effector link.

            Parameters:
                group_name (str): The name of the move group to control. Default is "".
            """
        self.cartesian_plan = None
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planning_time(10.0)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.waypoints = []
        self.orientation_status = False
        self.pose_status = False
        self.goal_tolerance = 0.005 # 5mm
        self.group_name = group_name

        
    def getJointStates(self):
        """
            Get the current joint states of the UR3 robot.
            Returns:
                list: A list containing the current joint values of the robot.
            """
        return self.move_group.get_current_joint_values()

    def getJointLimits(self):
        """
            Get the joint limits of the UR3 robot.
            Returns:
                dict: A dictionary containing the joint limits for each joint of the robot.
            """
        return self.robot.Joint.Bounds()

    def setJointStates(self, setpoint):
        """
            Move the UR3 robot to the specified joint configuration.

            Args:
                setpoint (list): A list containing the desired joint values for the robot.

            Returns:
                bool: True if the robot reached the target joint configuration within tolerance, False otherwise.
            """
        self.move_group.go(setpoint, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return all_close(setpoint, self.getJointStates(), 0.01)
    
    def orientationStatus(self, orientation):
        """
        Check the status of the robot's orientation based on the given orientation.

        Args:
            orientation (list): A list representing the desired orientation [x, y, z, w] in quaternion format.

        Returns:
            bool: True if the robot's orientation matches the desired orientation, or if the components 
                in the orientation list are marked as "-", indicating no change is required.
                False otherwise.

        Note:
            This method compares the robot's current orientation with the desired orientation, 
            taking into account a goal tolerance specified for the class instance. The orientation 
            components defined as "-" in the input list will not be considered in the comparison.
        """
        self.orientation_status = all(ori == '-' or (ori - ori_val) <= self.goal_tolerance for ori, ori_val in zip(orientation, self.getOrientation()))
        if self.orientation_status == True:
            rospy.loginfo("{} reached desired Orientation".format(self.group_name))
        else:
            rospy.logerr("{} did not reach desired Orientation".format(self.group_name))
        return self.orientation_status

    def poseStatus(self, pose):
        """
        Check the status of the robot's pose based on the given position.

        Args:
            pose (list): A list representing the desired position [x, y, z] in Cartesian coordinates.

        Returns:
            tuple: A tuple containing two elements:
                - bool: True if the robot's pose matches the desired pose, or if the components 
                        in the pose list are marked as "-", indicating no change is required.
                        False otherwise.
                - list: A list of positions that failed the comparison (i.e., positions that exceeded the goal tolerance).
        """
        self.pose_status = all(pos == '-' or (pos - pos_val) <= self.goal_tolerance for pos, pos_val in zip(pose, self.getPose()))
        failed_positions = [(i, pos) for i, (pos, pos_val) in enumerate(zip(pose, self.getPose())) if pos != '-' and (pos - pos_val) > self.goal_tolerance]
        if self.pose_status:
            rospy.loginfo("{} reached desired Pose".format(self.group_name))
        else:
            error_msg = "{} did not reach desired Pose. Failed positions: ".format(self.group_name)
            error_msg += ', '.join("{} = {}".format('xyz'[i], pos) for i, pos in failed_positions)
            rospy.logerr(error_msg)
        return self.pose_status, failed_positions

    def planCartesianPath(self):
        """
            Plan a cartesian path based on the stored waypoints.

            Returns:
                tuple: A tuple containing the computed cartesian path plan and the fraction of waypoints executed
                       (tuple structure: (cartesian_plan, fraction)).
            """
        cartesian_plan, fraction = self.move_group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        return cartesian_plan, fraction

    def clearWaypoints(self):
        """
           Clear the stored waypoints used for planning cartesian paths.
           """
        self.waypoints = []

    def displayTrajectory(self, plan):
        """
            Display the given motion plan as a trajectory for visualization.

            Args:
                plan (moveit_msgs.msg.RobotTrajectory): The motion plan to be displayed.
            """
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def executeTrajectory(self, plan):
        """
            Execute the given motion plan.

            Args:
                plan (moveit_msgs.msg.RobotTrajectory): The motion plan to be executed.
            """
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def getPose(self):
        """
            Get the current cartesian position of the robot's end effector.

            Returns:
                list: A list containing the current cartesian position [x, y, z] of the end effector.
            """

        current_pose = [0.0, 0.0, 0.0]
        current_pose[0] = self.move_group.get_current_pose().pose.position.x
        current_pose[1] = self.move_group.get_current_pose().pose.position.y
        current_pose[2] = self.move_group.get_current_pose().pose.position.z

        return current_pose

    def getOrientation(self):
        """
            Get the current orientation of the robot's end effector as a quaternion.

            Returns:
                list: A list containing the current orientation [x, y, z, w] of the end effector.
            """
        current_ori = [0.0, 0.0, 0.0, 0.0]
        current_ori[0] = self.move_group.get_current_pose().pose.orientation.x
        current_ori[1] = self.move_group.get_current_pose().pose.orientation.y
        current_ori[2] = self.move_group.get_current_pose().pose.orientation.z
        current_ori[3] = self.move_group.get_current_pose().pose.orientation.w

        return current_ori

    def goToOrientation(self, orientation, tolerance = 0.005):
        """
            Move the UR3 robot's end effector to a specified orientation.

            Args:
                orientation (list): A list representing the orientation [x, y, z, w] in quaternion format.

            Returns:
                bool: True if the robot reaches the target orientation within tolerance, False otherwise.
            """
        pose = geometry_msgs.msg.Pose()
        pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        pose.position = self.move_group.get_current_pose().pose.position
        self.move_group.set_pose_target(pose)
        self.move_group.set_goal_tolerance(tolerance)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose, current_pose, 0.01), self.orientation_status(orientation)

    def goToPose(self, pose,tolerance = 0.005):
        """
            Move the UR3 robot's end effector to a specified position in Cartesian coordinates.

            Args:
                pose (list): A list representing the position [x, y, z] in Cartesian coordinates.

            Returns:
                bool: True if the robot reaches the target position within tolerance, False otherwise.
            """
        pose = geometry_msgs.msg.Pose()
        pose.orientation = self.move_group.get_current_pose().pose.orientation
        pose.position = Point(pose[0], pose[1], pose[2])
        self.move_group.set_pose_target(pose)
        self.move_group.set_goal_tolerance(tolerance)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose, current_pose, 0.01), self.pose_status(pose)

    def moveTo(self, orientation, pose, tolerance = 0.005):
        """
            Move the UR3 robot's end effector to a specified orientation and position.

            Args:
                orientation (list): A list representing the orientation [x, y, z, w] in quaternion format.
                pose (list): A list representing the position [x, y, z] in Cartesian coordinates.

            Returns:
                bool: True if the robot reaches the target orientation and position within tolerance, False otherwise.
            """
        pose = geometry_msgs.msg.Pose()
        pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        pose.position = Point(pose[0], pose[1], pose[2])
        self.move_group.set_pose_target(pose)
        self.move_group.set_goal_tolerance(tolerance)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose, current_pose, 0.01), self.orientation_status(orientation), self.pose_status(pose)
    
    def cartesianOrientationTo(self,orientation):
        """
            Move the UR3 robot's end effector along a cartesian path with optional orientation changes.

            Args:
                orientation (list): A list representing the orientation [x, y, z, w] in quaternion format,
                                    with "-" indicating no change in the respective component.

            Note:
                The method updates the orientation components of the robot's pose based on the specified waypoints,
                where orientation components defined by "orientation" are optional (if "-" is provided, it means no change
                in orientation).
            """
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        if orientation[0] != "-":
            wpose.orientation.x = orientation[0]

        if orientation[1] != "-":
            wpose.orientation.y = orientation[1]

        if orientation[2] != "-":
            wpose.orientation.z = orientation[2]

        if orientation[3] != "-":
            wpose.orientation.w = orientation[3]
        waypoints.append(copy.deepcopy(wpose))
        (planned_path, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        self.displayTrajectory(planned_path)
        self.executeTrajectory(planned_path)

        return self.orientationStatus(orientation)

    def cartesianPoseTo(self, pose=["-", "-", "-"], relative=False, x=0.0, y=0.0, z=0.0):
        """
        Move the UR3 robot's end effector to a specified Cartesian pose.

        Args:
            pose (list, optional): A list representing the position [x, y, z] in Cartesian coordinates.
                                Use "-" for any component you want to keep unchanged (default: ["-", "-", "-"]).
            relative (bool, optional): If True, the provided x, y, and z values are interpreted as relative position
                                    changes to the current end effector position. If False, the provided pose is treated
                                    as an absolute position (default: False).
            x (float, optional): The change in the x-coordinate if `relative=True` (default: 0.0).
            y (float, optional): The change in the y-coordinate if `relative=True` (default: 0.0).
            z (float, optional): The change in the z-coordinate if `relative=True` (default: 0.0).

        Returns:
            tuple: A tuple containing two boolean values representing the status of reaching the desired orientation and pose.
                The first value indicates the orientation status, and the second value indicates the pose status.

        Note:
            The method moves the UR3 robot's end effector along a Cartesian path, where the position components defined by
            "pose" are optional (if "-" is provided, it means no change in position). If `relative=True`, the changes in
            x, y, and z are interpreted as relative position changes. If `relative=False`, the provided pose is treated as
            an absolute position.
        """
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose

        if pose[0] != "-":
            if relative:
                wpose.position.x += x
            else:
                wpose.position.x = pose[0]

        if pose[1] != "-":
            if relative:
                wpose.position.y += y
            else:
                wpose.position.y = pose[1]

        if pose[2] != "-":
            if relative:
                wpose.position.z += z
            else:
                wpose.position.z = pose[2]

        waypoints.append(copy.deepcopy(wpose))
        (planned_path, fraction) = move_group.compute_cartesian_path(waypoints, 0.05, 0.0)
        self.displayTrajectory(planned_path)
        self.executeTrajectory(planned_path)

        return self.poseStatus(pose)


    def cartesianMoveTo(self, orientation, pose):
        """
            Move the UR3 robot's end effector along a cartesian path with optional orientation and position changes.

            Args:
                orientation (list): A list representing the orientation [x, y, z, w] in quaternion format,
                                    with "-" indicating no change in the respective component.
                pose (list): A list representing the position [x, y, z] in Cartesian coordinates,
                            with "-" indicating no change in the respective component.

            Note:
                The method combines cartesianOrientationTo() and cartesianPoseTo() methods to update both the orientation
                and position components of the robot's pose based on the specified waypoints.
            """
        return self.cartesianOrientationTo(orientation), self.cartesianPoseTo(pose)

    
def main():
    try: 
        rospy.init_node("ur3_driver")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()