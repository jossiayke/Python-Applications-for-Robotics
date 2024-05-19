#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64


class MySubscriber:
    """
    A subscriber class to read messages from the odom topic.

    Attributes:
        _anonymous (bool): flag to create an anonymous node

    """
    def __init__(self, anonymous=False):
        """
        Initialize object attributes.

        Args:
            anonymous: Flag to check whether or not we need to create 
            anonymous nodes.
            anonymous=True will give a unique id to the node
            anonymous=True allows you to create multiple instances of this node

        Returns:
            None
        """
        self._anonymous = anonymous
        rospy.init_node("counter_sub", anonymous=self._anonymous)
        rospy.Subscriber("counter", Int64, self.callback)
       
        rospy.spin()


    def callback(self, msg):
        """
        Outputs received message data from counter topic
        
        Args:
            msg: Messages received on the Topic
        """
   
        rospy.loginfo("Receiving number [{}]"
                      .format(msg.data))
        

    
