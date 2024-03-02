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
Update Blackboard Variables
"""

##############################################################################
# Imports
##############################################################################

from typing import List

import py_trees
import rclpy.qos
import rclpy

##############################################################################
# Behaviours
##############################################################################


class TaskHandler(py_trees.behaviour.Behaviour):
    """
    This behaviour is a template behaviour to update blackboard variables.
    """

    def __init__(self,
                 name: str,
                 blackboard_variables: List[str],
                 qos_profile: rclpy.qos.QoSProfile,
                 ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        # Write access to blackboard variable
        self.blackboard_variables = blackboard_variables
        for blackboard_variable in self.blackboard_variables:
            key = blackboard_variable.split('.')[0]  # in case it is nested
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.WRITE
            )
        
        self.qos_profile = qos_profile


    def setup(self, **kwargs):
        """
        Setup the behaviour.
        
        Initalise Blackboard Variables
        
        E.g.
        self.blackboard.temperature = 0.0
        """
        try:
            self.node = kwargs['node']
            
            # USER DEFINED
            self.blackboard.temperature = 0.0
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
    def update(self):
        """
        Update Blackboard Variables
        
        E.g.
        
        self.blackboard.temperature += 1.0
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        # USER DEFINED
        self.blackboard.temperature += 1.0
        
        return py_trees.common.Status.SUCCESS