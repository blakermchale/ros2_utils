#!/usr/bin/env python3
'''
Log Node
========

Class that extends rclpy Node and adds the ability to publish all logs to a topic.
'''
import os
from rclpy.impl import rcutils_logger
from rcl_interfaces.msg import Log
from rclpy.logging import LoggingSeverity
from rclpy.node import Node


class LogNode(Node):
    def __init__(self, node_name:str, log_topic:str, **kwargs):
        """Node for logging and publishing messages to a topic.

        Args:
            node_name (str): A name to give to this node. Validated by :func:`validate_node_name`.
            log_topic (str): Topic to log this nodes information.
        """
        # Ignore this file when looking for which file, function, and line number the log is coming
        #  from
        log_ignore = os.path.realpath(__file__)
        if log_ignore not in rcutils_logger._internal_callers:
            rcutils_logger._internal_callers.extend([log_ignore])
        super().__init__(node_name, **kwargs)
        self._log_name = "Mission Log"
        self._pub_log = self.create_publisher(Log, log_topic, 10)

    def publish_log(self, message:str, log_level:LoggingSeverity, **kwargs):
        """Method to log and publish a log message to the /log topic and the terminal. 

        Args:
            message (str): Message to log.
            log_level (LoggingSeverity): Log level of the message.
        """
        log = Log()
        log.stamp = self.get_clock().now().to_msg()
        log.level = log_level
        log.name = self._log_name
        log.msg = message
        self.get_logger().log(message, log_level, **kwargs)
        self._pub_log.publish(log)

    def publish_info(self, message:str, **kwargs):
        """Publish log INFO message to node topic.

        Args:
            message (str): Message to log.
        """
        self.publish_log(message, LoggingSeverity.INFO, **kwargs)

    def publish_debug(self, message:str, **kwargs):
        """Publish log DEBUG message to node topic.

        Args:
            message (str): Message to log.
        """
        self.publish_log(message, LoggingSeverity.DEBUG, **kwargs)

    def publish_warn(self, message:str, **kwargs):
        """Publish log WARN message to node topic.

        Args:
            message (str): Message to log.
        """
        self.publish_log(message, LoggingSeverity.WARN, **kwargs)

    def publish_error(self, message:str, **kwargs):
        """Publish log ERROR message to node topic.

        Args:
            message (str): Message to log.
        """
        self.publish_log(message, LoggingSeverity.ERROR, **kwargs)

    def publish_fatal(self, message:str, **kwargs):
        """Publish log FATAL message to node topic.

        Args:
            message (str): Message to log.
        """
        self.publish_log(message, LoggingSeverity.FATAL, **kwargs)
