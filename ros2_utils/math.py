#!/usr/bin/env python3
import numpy as np
from enum import IntEnum, auto


class AxesFrame(IntEnum):
    LHAND=0  # x fwd, y right, z up
    RHAND=auto()  # x fwd, y left, z up
    ULHAND=auto()  # upside down left, x fwd, y right, z down
    URHAND=auto()  # upside down right, x fwd, y right, z down


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angular_dist(start, end):
    start = wrap_to_pi(start)
    end = wrap_to_pi(end)
    d = end - start
    if d > np.pi:
        d -= 2 * np.pi
    elif d < -np.pi:
        d += 2 * np.pi
    return d


def convert_axes(x, y, z, roll, pitch, yaw, in_axes: AxesFrame, out_axes: AxesFrame):
    if (in_axes == AxesFrame.URHAND and out_axes == AxesFrame.RHAND) \
        or (in_axes == AxesFrame.RHAND and out_axes == AxesFrame.URHAND) \
        or (in_axes == AxesFrame.ULHAND and out_axes == AxesFrame.LHAND) \
        or (in_axes == AxesFrame.LHAND and out_axes == AxesFrame.ULHAND):
        return x, -y, -z, roll, pitch, -yaw
    elif (in_axes == AxesFrame.RHAND and out_axes == AxesFrame.LHAND) \
        or (in_axes == AxesFrame.LHAND and out_axes == AxesFrame.RHAND) \
        or (in_axes == AxesFrame.URHAND and out_axes == AxesFrame.ULHAND) \
        or (in_axes == AxesFrame.ULHAND and out_axes == AxesFrame.URHAND):
        return x, -y, z, roll, pitch, yaw
    elif (in_axes == AxesFrame.RHAND and out_axes == AxesFrame.RHAND) \
        or (in_axes == AxesFrame.LHAND and out_axes == AxesFrame.LHAND) \
        or (in_axes == AxesFrame.ULHAND and out_axes == AxesFrame.ULHAND) \
        or (in_axes == AxesFrame.URHAND and out_axes == AxesFrame.URHAND):
        return x, y, z, roll, pitch, yaw
    else:
        raise ValueError(f"In axes {in_axes.name} and out axes {out_axes.name} are not supported")
