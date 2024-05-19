#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64


class MyPublisher:
    """
    A publisher class to build and publish messages to the cmd_vel topic.

    Attributes:
        _anonymous (bool): flag to create an anonymous node
        _publisher (Publisher): Publisher object
        _message (INT64): Int64 object for std_msgs/Int64 messages
        _rate (rospy.timer.Rate)
        _increment (INT64): holds value added to or subtracted from saved data
        _operation (str): saves if arithmetic type (addition or subtraction)

    """

    def __init__(self, anonymous=False, rate=1):
        """
        Initialize object attributes.

        Args:
            anonymous: Flag to check whether or not we need to create 
            anonymous nodes.
            anonymous=True will give a unique id to the node
            anonymous=True allows you to create multiple instances of this node

            rate: Rate at which messages are published on a topic.

        Returns:
            None
        """
        self._anonymous = anonymous
        rospy.init_node("counter_pub", anonymous=self._anonymous)
        self._publisher = rospy.Publisher("counter", Int64, queue_size=10)
        self._message = Int64()
        self._message.data = 1
        self._rate = rospy.Rate(rate)
        self._increment = 0
        self._operation = ''
        
        if rospy.has_param('/publisher_frequency'):
            self._rate = rospy.Rate(rospy.get_param('/publisher_frequency'))
            
        if rospy.has_param('/math_operation/operation_type'):
            self._operation = rospy.get_param('/math_operation/operation_type')
            
        if rospy.has_param('/math_operation/constant'):
            self._increment = rospy.get_param('/math_operation/constant')

    def arithmetic(self):
        """
        Build and publish Messages to the counter Topic
        
        Args:
            None

        Returns:
            None
        
        """
        
        
        rospy.loginfo("Self Increment is {}" .format(self._increment))
        rospy.loginfo("Publishing number starts from [{}]"
                          .format(self._message.data))
        # Publish messages in a loop (until we do Ctrl-C)
        while not rospy.is_shutdown():
            if self._operation == "addition":
                self._message.data += self._increment
            elif self._operation == "subtraction":
                self._message.data -= self._increment
            else:
                rospy.loginfo("Incorrect arithmetic given, so data not changing.")

            rospy.loginfo("Publishing number [{}]"
                          .format(self._message.data))
            self._publisher.publish(self._message)
            self._rate.sleep()
