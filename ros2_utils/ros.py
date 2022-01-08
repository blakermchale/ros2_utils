from typing import Union
from .math import convert_axes, AxesFrame
from geometry_msgs.msg import Pose, Transform, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from .structs import NpVector4, NpPose, NpVector3, NpTwist
import numpy as np


def convert_axes_from_msg(msg: Union[Pose, Transform, Twist, Odometry], in_axes: AxesFrame, out_axes: AxesFrame):
    """Converts ROS message coordinate frame."""
    if isinstance(msg, Pose):
        q = NpVector4.ros(msg.orientation)
        x, y, z, roll, pitch, yaw = convert_axes(msg.position.x, msg.position.y, msg.position.z, q.roll, q.pitch, q.yaw, in_axes, out_axes)
        return NpPose(NpVector3.xyz(x, y, z), NpVector4.rpy(roll, pitch, yaw)).get_msg()
    elif isinstance(msg, Transform):
        q = NpVector4.ros(msg.rotation)
        x, y, z, roll, pitch, yaw = convert_axes(msg.translation.x, msg.translation.y, msg.translation.z, q.roll, q.pitch, q.yaw, in_axes, out_axes)
        return NpPose(NpVector3.xyz(x, y, z), NpVector4.rpy(roll, pitch, yaw)).get_tf_msg()
    elif isinstance(msg, Twist):
        x, y, z, roll, pitch, yaw = convert_axes(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z, in_axes, out_axes)
        return NpTwist(NpVector3.xyz(x, y, z), NpVector3.xyz(roll, pitch, yaw)).get_msg()
    elif isinstance(msg, Odometry):
        out_msg = Odometry()
        pose_msg = msg.pose.pose
        twist_msg = msg.twist.twist
        q = NpVector4.ros(pose_msg.orientation)
        x, y, z, roll, pitch, yaw = convert_axes(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, q.roll, q.pitch, q.yaw, in_axes, out_axes)
        out_msg.pose.pose = NpPose(NpVector3.xyz(x, y, z), NpVector4.rpy(roll, pitch, yaw)).get_msg()
        x, y, z, roll, pitch, yaw = convert_axes(twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z, in_axes, out_axes)
        out_msg.twist.twist = NpTwist(NpVector3.xyz(x, y, z), NpVector3.xyz(roll, pitch, yaw)).get_msg()
        return out_msg
    else:
        raise ValueError(f"ROS message type {type(msg)} is not supported")


def msg_contains_nan(msg: Union[Point, Vector3]):
    """Checks if ROS message contains any nans."""
    if isinstance(msg, (Point, Vector3)):
        return np.isnan(msg.x) or np.isnan(msg.y) or np.isnan(msg.z)
    else:
        raise ValueError(f"ROS message type {type(msg)} is not supported")
