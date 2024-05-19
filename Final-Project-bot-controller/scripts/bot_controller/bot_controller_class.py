#!/usr/bin/env python3

from genericpath import exists
import rospy
import sys
import tf

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt, atan2, pi
import tf2_ros
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel
from enpm809e_msgs.msg import PartInfo, PartInfos, LogicalCameraImage, Model
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Pose, TransformStamped, Twist, Point
from bot_controller.part_class import Part


class BotController(object):
    """
    A controller class to drive a turtlebot in Gazebo.
    
    Args:
        self._is_part_info_gathered: (bool) - sets True if parts info obtained
        self._part_list: (list) - stores parts qued to be moved
        self._camera_parts: (list) - stores parts sensed by cameras
        self._cam_dict: (dict)  - stoes camera number with associated part
        self._queried_camera_1: (bool) - sets True if camera data has been processed
        self._queried_camera_2: (bool) - sets True if camera data has been processed
        self._queried_camera_3: (bool) - sets True if camera data has been processed
        self._queried_camera_4: (bool) - sets True if camera data has been processed
        self._rate: (rospy.Rate) - used to assign rate to publishing data
        self._robot_name: (str) -  stores robot name
        self._velocity_msg: (Twist)  -  used to store linear and angular robot velocity
        self._kp_linear: (float) -  stores linear proportional controller
        self._kp_angular: (float) -  stores angular proportional controller
        self._current_x_pos: (float) - stores current robot x position
        self._current_y_pos: (float) - stores current robot y position
        self._current_orientation: (float) - stores robot current orientation as quaternion
        self._initial_orientation: (float) - stores robot initial orientation as quaternion
        self._goal_reached: (bool)  -  sets to True when robot reaches goal
        self._velocity_publisher: (Twist)  -  publishes robot velocity to cmd_vel
        self._camera1_msg: (Rosmsg) -  stores part type, pose, and camera pose info
        self._camera2_msg: (Rosmsg) -  stores part type, pose, and camera pose info
        self._camera3_msg: (Rosmsg) -  stores part type, pose, and camera pose info
        self._camera4_msg: (Rosmsg) -  stores part type, pose, and camera pose info
        
    Return:
        None
    """

    def __init__(self, rate=10):
        """
        Initializes BotController object with the following attributes

        Args:
            rate (int, optional): _description_. Defaults to 10.
            
        Return:
            None
        """
        
        rospy.init_node('bot_controller')
        rospy.loginfo('Press Ctrl c to exit')
        self._is_part_info_gathered = False
        self._part_list = []
        self._camera_parts = []
        self._cam_dict = {}

        #  used to check whether a camera data has been processed
        self._queried_camera_1 = False
        self._queried_camera_2 = False
        self._queried_camera_3 = False
        self._queried_camera_4 = False
        
        self._rate = rospy.Rate(rate)
        self._robot_name = 'waffle'
        self._velocity_msg = Twist()
        
        #  gains for the proportional controller
        self._kp_linear = 0.2
        self._kp_angular = 0.2

        #  default velocities for going in a straight line
        self._velocity_msg.linear.x = 0.1
        self._velocity_msg.angular.z = 0.1

        # current pose of the robot
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._initial_orientation = None

        #  used to check whether the goal has been reached
        self._goal_reached = False

        # Publishers
        self._velocity_publisher = rospy.Publisher(
            'cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.subscribe()
    
        # Save parts queue from /part_infos
        self._parts_list = self._part_infos_msg.part_infos
        
        # Execute action to deliver parts to their drop location
        self.delivery()
        rospy.spin()

    @staticmethod
    def compute_distance(x1, y1, x2, y2):
        """Compute distance between 2 points

        Args:
            x1 (float): x position of the first point
            y1 (float): y position of the first point
            x2 (float): x position of the second point
            y2 (float): y position of the second point

        Returns:
            float: distance between 2 points
        """
        return sqrt(((x2-x1)**2) + ((y2-y1)**2))

    @staticmethod
    def get_model_pose(model_name="waffle"):
        """
        Get the pose of the turtlebot in the environment.
        This function is used by the method attach_part

        Args:
            model_name (str, optional): Name of the robot model in Gazebo. Defaults to "waffle".

        Raises:
            RuntimeError: Raised when the robot model cannot be retrieved

        Returns:
            Pose: Pose of the robot in the Gazebo world
        """
        poll_rate = rospy.Rate(1)
        for i in range(10):
            model_states = rospy.wait_for_message(
                '/gazebo/model_states', ModelStates, 1)
            if model_name in model_states.name:
                model_pose = model_states.pose[model_states.name.index(
                    model_name)]
                break
            poll_rate.sleep()
        else:
            raise RuntimeError('Failed to get ' + model_name + ' model state')
        return model_pose

    def subscribe(self):
        """
        Subscribe to /odom, all four cameras, and /part_infos
        
        Args:
            None
            
        Return:
            None 
        """
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.Subscriber("/logical_camera/camera_1",
                         LogicalCameraImage, self.camera_1_callback)
        self._camera1_msg = rospy.wait_for_message('/logical_camera/camera_1', LogicalCameraImage)
        rospy.loginfo(self._camera1_msg)
        
        if self._camera1_msg is not None:
            model = self._camera1_msg.models[0]
            self._cam_dict[str(model.type)] = '1'
        else:
            self._camera_parts.append(None)

        rospy.Subscriber("/logical_camera/camera_2",
                         LogicalCameraImage, self.camera_2_callback)
        self._camera2_msg = rospy.wait_for_message('/logical_camera/camera_2', LogicalCameraImage)
        
        rospy.loginfo(self._camera2_msg)
        if self._camera2_msg is not None:
            model = self._camera2_msg.models[0]
            self._cam_dict[str(model.type)] = '2'
        else:
            self._camera_parts.append(None)
            
        rospy.Subscriber("/logical_camera/camera_3",
                         LogicalCameraImage, self.camera_3_callback)
        self._camera3_msg = rospy.wait_for_message('/logical_camera/camera_3', LogicalCameraImage)
        rospy.loginfo(self._camera3_msg)
        
        if self._camera3_msg is not None:
            model = self._camera3_msg.models[0]
            self._cam_dict[str(model.type)] = '3'
        else:
            self._camera_parts.append(None)
            
        rospy.Subscriber("/logical_camera/camera_4",
                         LogicalCameraImage, self.camera_4_callback)
        self._camera4_msg = rospy.wait_for_message('/logical_camera/camera_4', LogicalCameraImage)
        rospy.loginfo(self._camera4_msg)
        
        if self._camera4_msg is not None:
            model = self._camera4_msg.models[0]
            self._cam_dict[str(model.type)] = '4'
        else:
            self._camera_parts.append(None)
            
        self._part_infos_msg = rospy.wait_for_message('/part_infos', PartInfos)
        rospy.loginfo(self._part_infos_msg)
        
    def attach_part(self, part_type):
        """
        Attach a Gazebo model to the turtlebot
        
        Args:
        
            part_type: (str) - type of part to attach
            
        Return:
        
            None
        """
        part_name = part_type+"_1"
        rospy.loginfo(part_name)

        pause_physics_client = rospy.ServiceProxy(
            '/gazebo/pause_physics', Empty)
        unpause_physics_client = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)
        spawn_sdf_model_client = rospy.ServiceProxy(
            '/gazebo/spawn_sdf_model', SpawnModel)
        link_attacher_client = rospy.ServiceProxy(
            '/link_attacher_node/attach', Attach)
        delete_model_client = rospy.ServiceProxy(
            'gazebo/delete_model', DeleteModel)
        
        # Remove from Gazebo
        delete_model_client(part_type+"_0")

        # Compute part pose based on current robot pose
        part_pose = BotController.get_model_pose(self._robot_name)
        rospy.loginfo(part_pose.position.x)
        rospy.loginfo(part_pose.position.y)
        rospy.logwarn(self._robot_name)
        # part_pose.position.x -= 0.05
        part_pose.position.y += 0.5
        part_pose.position.z += 0.3

        model_name = "model://"+part_type+"_ariac"
        rospy.loginfo("Model: {}".format(model_name))
        
        # Compute spawn model request
        spawn_request = SpawnModelRequest()
        spawn_request.model_name = part_name
        
        # spawn_request.model_name = "assembly_pump_yellow_0"
        spawn_request.model_xml = '<sdf version="1.6"> \
        <world name="default"> \
            <include> \
                <uri>{0}</uri> \
            </include> \
            </world> \
        </sdf>'.format(model_name)
        spawn_request.robot_namespace = self._robot_name
        spawn_request.initial_pose = part_pose

        # Pause Gazebo
        try:
            pause_physics_client()
        except rospy.ServiceException as e:
            rospy.logerr("Pause physics service call failed: %s", e)

        spawn_sdf_model_client(spawn_request)

        #  Attach the part to the robot
        try:
            link_attacher_client(
                self._robot_name, 'base_footprint', part_name, 'link')
        except rospy.ServiceException as e:
            rospy.logerr("Link attacher service call failed: %s", e)

        #  Unpause Gazebo
        try:
            unpause_physics_client()
        except rospy.ServiceException as e:
            rospy.logerr("Unpause physics service call failed: %s", e)

    def detach_part(self, part_type):
        """
        Detach a model from the turtlebot
        
        Arg:
            part_type: (str) - name of part type to be detached
            
        Return:
            None
        """
        
        detach_client = rospy.ServiceProxy('/link_attacher_node/detach',
                                           Attach)
        detach_client.wait_for_service()

        part_name = part_type+"_1"

        # Build the detach request
        rospy.loginfo("Detaching part from robot")
        req = AttachRequest()
        req.model_name_1 = self._robot_name
        req.link_name_1 = "base_footprint"
        req.model_name_2 = part_name
        req.link_name_2 = "link"

        detach_client.call(req)

    def camera_1_callback(self, msg):
        """
        Callback for Topic /logical_camera/camera_1
        
        Arg:
            msg: (str) - message from /part_info
            
        Return:
            None
        """
        
        model = msg.models
        
        if not self._queried_camera_1:
           
            camera_part = Part(
                    model[0].type, "camera_1_"+model[0].type+"_0_frame", model[0].pose)
            self._camera_parts.append(camera_part)
            self._queried_camera_1 = True

    def camera_2_callback(self, msg): 
        """
        Callback for Topic /logical_camera/camera_2
        
        Arg:
            msg: (str) - message from /part_info
            
        Return:
            None
        """
        
        model = msg.models
        
        if not self._queried_camera_2:
            
            camera_part = Part(
                    model[0].type, "camera_2_"+ model[0].type+"_0_frame", model[0].pose)
            self._camera_parts.append(camera_part)
            self._queried_camera_2 = True
            
    def camera_3_callback(self, msg): 
        """
        Callback for Topic /logical_camera/camera_3
        
        Arg:
            msg: (str) - message from /part_info
            
        Return:
            None
        """
        
        model = msg.models
        
        if not self._queried_camera_3:
            
            camera_part = Part(
                    model[0].type, "camera_3_"+ model[0].type+"_0_frame", model[0].pose)
            self._camera_parts.append(camera_part)
            self._queried_camera_3 = True
            
    def camera_4_callback(self, msg):
        """
        Callback for Topic /logical_camera/camera_4
        
        Arg:
            msg: (str) - message from /part_info
            
        Return:
            None
        """
        
        model = msg.models
        
        if not self._queried_camera_4:
            
            camera_part = Part(
                    model[0].type, "camera_4_"+ model[0].type+"_0_frame", model[0].pose)
            self._camera_parts.append(camera_part)
            self._queried_camera_4 = True
            
    def odom_callback(self, msg):
        """
        Callback function for the Topic odom
        
        Args:
            msg (nav_msgs/Odometry): Odometry message
            
        Return:
            None
        """
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self._current_x_pos = msg.pose.pose.position.x
        self._current_y_pos = msg.pose.pose.position.y
        self._current_orientation = euler_from_quaternion(quaternion)

    def get_transform(self, source, target):
        """Get the transform between two frames

        Args:
            source (str): source frame
            target (str): target frame

        Returns:
            geometry_msgs/Pose: The pose of the source frame in the target frame
        """
        
        tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tf_buffer)

        transform_stamped = TransformStamped()

        rospy.logwarn("Source: {}".format(source))
        rospy.logwarn("Target: {}".format(target))

        try:
            transform_stamped = tf_buffer.lookup_transform(
                target,
                source,
                rospy.Time(),
                rospy.Duration(1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")

        pose = Pose()
        pose.position = transform_stamped.transform.translation
        rospy.logwarn("Transform: {}".format(pose))
        pose.orientation = transform_stamped.transform.rotation
        return pose

    def go_to_goal(self, goal_x, goal_y):
        """
        Make the robot reach a 2D goal using a proportional controller
        
        Args:
            goal_x (float): x position
            goal_y (float): y position
            
        Return:
            None
        """
        rospy.loginfo("Go to goal ({}, {})".format(goal_x, goal_y))

        distance_to_goal = BotController.compute_distance(
            self._current_x_pos, self._current_y_pos, goal_x, goal_y)

        while not rospy.is_shutdown():
            move_cmd = Twist()
            if distance_to_goal > 0.1:
                distance_to_goal = BotController.compute_distance(self._current_x_pos,
                                                                  self._current_y_pos, goal_x, goal_y)
                # compute the heading
                angle_to_goal = atan2(
                    goal_y - self._current_y_pos, goal_x - self._current_x_pos)

                # rospy.loginfo("Distance to goal: {}".format(distance_to_goal))
                # rospy.loginfo("Angle to goal: {}".format(angle_to_goal))

                # Make the robot rotate to face the goal
                if angle_to_goal < 0:
                    angle_to_goal = 2 * pi + angle_to_goal

                # compute relative orientation between robot and goal
                w = angle_to_goal - self._current_orientation[2]
                if w > pi:
                    w = w - 2 * pi

                # proportional control for angular velocity
                move_cmd.angular.z = self._kp_angular * w

                # turtlebot max angular velocity is 2.84 rad/s
                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                # proportional control for linear velocity
                # turtlebot max linear velocity is 0.22 m/s
                move_cmd.linear.x = min(
                    self._kp_linear * distance_to_goal, 0.6)

                self._velocity_publisher.publish(move_cmd)
                self._rate.sleep()
            else:
                rospy.loginfo("Goal reached")
                self._goal_reached = True
                self.run(0, 0)
                return True

    def run(self, linear, angular):
        """
        Publish linear and angular velocities to cmd_vel Topic.
        
        Args:
            linear (float): linear velocity
            angular (float): angular velocity
            
        Return:
            None
        """
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

    def myhook(self):
        """
        Function to call when shutting down a Node
        
        Args:
            None
            
        Return:
            None
        """
        rospy.loginfo("shutdown time!")

            
    def delivery(self):
        """
        Perform task of sensing and delivering parts
        to their intended goal
        
        Args:
           None
            
        Return:
            None
        
        """
        
        # Identify number of parts in queue
        num_parts = len(self._parts_list)
        
        for i in range(num_parts):
            part_name = self._parts_list[i].part_type
            rospy.loginfo(part_name)
            drop_location = self._parts_list[i].drop_location
                 
            if part_name in self._cam_dict.keys():
                
                # Find camera looking at part
                camera_prt = self._cam_dict[part_name]
                part_by_cam = self._camera_parts[int(camera_prt)-1]
                
                rospy.loginfo("Part %s is located near camera %s", part_name, camera_prt)
                
                new_pose = self.get_transform(part_by_cam._frame,"odom")
                                                            
                goal_x = new_pose.position.x
                goal_y = new_pose.position.y
                
                # Move robot to location of part
                if self.go_to_goal(goal_x, goal_y):
                    rospy.logwarn("Action completed")
                    rospy.loginfo("Robot reached pickup location reached")
                
                # Attach part to robot
                self.attach_part(part_name)
                
                # Move part to drop location
                if self.go_to_goal(drop_location.x, drop_location.y):
                    rospy.logwarn("Action completed")
                    rospy.loginfo("Drop location reached")
                
                # Detach part from robot
                self.detach_part(part_name)
            
            else:
                rospy.WARN("Part not found near any of the four cameras")
                if i == num_parts-1:
                    rospy.logfatal("Given Parts can not be found")
                    rospy.logwarn("Terminating Program!")
                    rospy.on_shutdown(self.myhook)
                    sys.exit(1)
                else:
                    continue                
            
        # Return robot to initial position, since parts have been delivered
        if self.go_to_goal(0, 0):
            rospy.logwarn("Action completed")
            rospy.on_shutdown(self.myhook)
            sys.exit(1)

