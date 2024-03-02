#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros/devel/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
Convenience behaviours for publishing ROS messages.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rclpy.qos

##############################################################################
# Behaviours
##############################################################################


class TaskHandler(py_trees.behaviour.Behaviour):
    """
    This behaviour publishes a message to a topic using a blackboard variable.
    """
    def __init__(self,
                 name: str,
                 topic_name: str,
                 topic_type,
                 blackboard_variable: str,
                 qos_profile: rclpy.qos.QoSProfile,
                 ):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard_variable = blackboard_variable
        self.key = blackboard_variable.split('.')[0]  # in case it is nested
        self.blackboard.register_key(
            key=self.key,
            access=py_trees.common.Access.READ
        )
        self.publisher = None
        self.qos_profile = qos_profile
        self.node = None

    def setup(self, **kwargs):
        """
        Initialises the publisher.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        self.publisher = self.node.create_publisher(
            msg_type=self.topic_type,
            topic=self.topic_name,
            qos_profile=self.qos_profile
        )

    def update(self):
        """
        Publish the variable from the blackboard.

        Raises:
            TypeError if the blackboard variable is not of the required type

        Returns:
            :data:`~py_trees.common.Status.FAILURE` (variable does not exist on the blackboard) or :data:`~py_trees.common.Status.SUCCESS` (published)
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        try:
            self.logger.info("publishing: {}".format(self.blackboard.get(self.blackboard_variable)))
            msg = self.topic_type()
            msg.data = self.blackboard.get(self.blackboard_variable)
            self.publisher.publish(msg)
            self.feedback_message = "published"
            return py_trees.common.Status.SUCCESS
        except KeyError:
            self.feedback_message = "nothing to publish"
            return py_trees.common.Status.FAILURE