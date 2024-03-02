#!/usr/bin/env python3

import operator

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

import std_msgs.msg 
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import task_publish_var
import task_update_vars

class BT(rclpy.node.Node):
    def __init__(self):  
        super().__init__('BT_Node',
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)
        self.latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        root = self.create_root()
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            unicode_tree_debug=True
        )
        try:
            tree.setup(timeout=15.0)
        except py_trees_ros.exceptions.TimedOutError as e:
            console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
            tree.shutdown()
            rclpy.shutdown()
            sys.exit(1)
        except KeyboardInterrupt:
            # not a warning, nor error, usually a user-initiated shutdown
            console.logerror("tree setup interrupted")
            tree.shutdown()
            rclpy.shutdown()
            sys.exit(1)

        tree.tick_tock(period_ms=1000.0)

        try:
            rclpy.spin(tree.node)
        except KeyboardInterrupt:
            pass

        tree.shutdown()
        rclpy.shutdown()
        
    def _create_topics2bb(self):
        """
        Create a behaviour that reads the event output from a topic and writes it to the blackboard.
        """
        topics2bb = py_trees.composites.Sequence(name="Topics to Blackboard", memory=True)
        
        heaton_sub = py_trees_ros.subscribers.EventToBlackboard(
            name="heaton Subscriber",
            topic_name="copilot/heaton",
            variable_name="heaton",
            qos_profile=self.latching_qos
        )
        topics2bb.add_child(heaton_sub)
        
        heatoff_sub = py_trees_ros.subscribers.EventToBlackboard(
            name="heatoff Subscriber",
            topic_name="copilot/heatoff",
            variable_name="heatoff",
            qos_profile=self.latching_qos
        )
        topics2bb.add_child(heatoff_sub)
        
        
        return topics2bb
    
    
    def _create_heaton_check(self):
        """
        Create a behaviour that checks if event has been inputted from ROS.
        """
        
        is_heaton = py_trees.behaviours.CheckBlackboardVariableValue(
            name="is_heaton?",
            check=py_trees.common.ComparisonExpression(
                variable="heaton",
                value=True,
                operator=operator.eq
            )
        )
        
        return is_heaton
    
    def _create_heatoff_check(self):
        """
        Create a behaviour that checks if event has been inputted from ROS.
        """
        
        is_heatoff = py_trees.behaviours.CheckBlackboardVariableValue(
            name="is_heatoff?",
            check=py_trees.common.ComparisonExpression(
                variable="heatoff",
                value=True,
                operator=operator.eq
            )
        )
        
        return is_heatoff
    
    
    
    def _create_tasks(self):
        """
        Create a behaviour that updates & writes the variables to the blackboard and publishes them to a ROS topic.
        """
        tasks = py_trees.composites.Sequence(name="Tasks", memory=True)
        update_temp = task_update_vars.TaskHandler(
            name="Update Temperature",
            blackboard_variables=["temperature",],
            qos_profile=self.latching_qos
        )
        tasks.add_child(update_temp)
        
        publish_temperature = task_publish_var.TaskHandler(
            name="Publish temperature",
            topic_name="copilot/temperature",
            topic_type=std_msgs.msg.Float32,
            blackboard_variable="temperature",
            qos_profile=self.latching_qos
        )
        tasks.add_child(publish_temperature)
        
        return tasks
    
    def create_root(self) -> py_trees.behaviour.Behaviour:
        """
        Create a basic tree for the monitor application.

        Note: This tree is an example and should be replaced with the actual tree for the application.
        """
        root = py_trees.composites.Parallel(
            name="BT Node",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        
        topics2bb = self._create_topics2bb()
        root.add_child(topics2bb)
        
        heaton_check = self._create_heaton_check()
        root.add_child(heaton_check)
        
        heatoff_check = self._create_heatoff_check()
        root.add_child(heatoff_check)
        
        tasks = self._create_tasks()
        root.add_child(tasks)
        
        return root

def main(args=None):
    print('Starting BT_Node')
    rclpy.init(args=args)
    node = BT()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    print('Stopping BT_Node')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()