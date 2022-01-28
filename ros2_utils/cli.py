#!/usr/bin/env python3
'''
Client
=======================

Common functions and classes for instantiating ROS clients and completing action calls.
'''
from typing import List, Type
from rclpy.duration import Duration
from rclpy.action.client import GoalStatus
from rclpy.executors import Executor
from rclpy.task import Future
from rclpy.node import Node
import functools
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from cmd2 import Cmd, Cmd2ArgumentParser, with_argparser
from rclpy.executors import MultiThreadedExecutor
from enum import IntEnum, auto
from rclpy.action.client import ActionClient


class NodeClient(Node):
    def __init__(self, node_name: str, executor: Executor, **kwargs):
        super().__init__(node_name, use_global_arguments=False, **kwargs)  #NOTE: need to disable global args to accept namespace from kwargs
        self.namespace = self.get_namespace().split("/")[-1]

        # Internal states
        self._timeout_sec = 10.0
        self._waiting_for_gh = False
        self.feedback = None

        # Goal handles
        self._goal_handles = {}

        self._executor = executor
        self._executor.add_node(self)
    
    ########################
    ## Helpers
    ########################
    def reset_actions(self):
        # self._goal_handles = {}
        self._waiting_for_gh = True
        self.feedback = None

    def _action_response(self, action_name: str, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._waiting_for_gh = False
            self.get_logger().error(f"Goal rejected for '{action_name}'")
            return
        self._goal_handles[action_name] = goal_handle
        self.get_logger().info(f"Goal accepted for '{action_name}'")


def setup_send_action(self: Node, action_cli: ActionClient, feedback_cb):
    """Decorator for sending an action with NodeClient."""
    def inner(func):
        if action_cli._action_name in self._goal_handles.keys():
            self.get_logger().error(f"`{action_cli._action_name}` is still being sent")
            return
        self.get_logger().info(f"entities{action_cli.get_num_entities()}")
        self.reset_actions()
        if not action_cli.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error(f"No action server available for `{action_cli._action_name}`")
            return
        goal = func()
        self.get_logger().info(f"Sending goal to `{action_cli._action_name}`")
        future = action_cli.send_goal_async(goal, feedback_callback=feedback_cb)
        future.add_done_callback(functools.partial(self._action_response, action_cli._action_name))
        return future
    return inner



# https://pymotw.com/2/cmd/
# https://pypi.org/project/cmd2/
class ClientShell(Cmd):
    """Generic client shell that handles cancelling actions. Assumes field `self.client` is set to a child of `NodeClient`."""
    def __init__(self, name, ClientObj: NodeClient, **kwargs) -> None:
        # print(kwargs)
        super().__init__(**kwargs)
        self.ClientObj = ClientObj
        client = self.ClientObj(MultiThreadedExecutor(), namespace=name)
        self.clients_archive = {name: client}
        self.client = client
        self.name = name

    _set_name_argparser = Cmd2ArgumentParser(description='Changes client to new vehicle name.')
    _set_name_argparser.add_argument('name', type=str, help='vehicle namespace')
    @with_argparser(_set_name_argparser)
    def do_set_name(self, opts):
        if opts.name not in self.clients_archive.keys():
            self.clients_archive[opts.name] = self.ClientObj(MultiThreadedExecutor(), namespace=opts.name)
        self.client = self.clients_archive[opts.name]
        self.name = opts.name

    def do_get_name(self, opts):
        """Gets current client vehicle name."""
        print(f"Vehicle name is '{self.client.namespace}'")

    def sigint_handler(self, signum: int, _) -> None:
        cancel_futures = []
        if self.client._waiting_for_gh:
            print("Cannot cancel until goal is retrieved")
            return
        for k, v in list(self.client._goal_handles.items()):
            if v.status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                print(f"Goal `{k}` already ended, removing")
                self.client._goal_handles.pop(k)
                continue
            cancel_futures.append(v.cancel_goal_async())
            print(f"\nCancelling `{k}`!")
        # Wait for all cancel requests to complete and the goals to be canceled
        if cancel_futures:
            return # Assume complete action call will exit once canceled
        super().sigint_handler(signum, _)

    def do_exit(self, args):
        """Exit shell."""
        print("Exiting")
        return True

    def default(self, inp):
        if inp in ["x", "q"]:
            return self.do_exit(inp)
        print("Default not implemented: {}".format(inp))

    do_EOF = do_exit

    @property
    def executor(self):
        return self.client.executor


class CompleteActionState(IntEnum):
    WAIT_GH=0
    WAIT_GH_FUTURE=auto()
    CHECK_STATUS=auto()
    END=auto()
    SUCCEEDED=auto()
    FAILURE=auto()
    CANCELED=auto()


def gh_state_machine(data):
    """Progresses goal handle state further when futures and results are found. Assumes the node that called the action inherits from NodeClient.

    Args:
        data (dict): Contains 'state' and 'future' from action call. Stores relevant data as state machine progresses.

    Returns:
        CompleteActionState: New state of state machine.
    """
    state = data["state"]
    if state == CompleteActionState.WAIT_GH:
        if data["future"].done():
            data["goal_handle"] = data["future"].result()
            data["action_name"] = data["goal_handle"]._action_client._action_name
            data["node"] = data["goal_handle"]._action_client._node
            data["node"]._waiting_for_gh = False
            return CompleteActionState.WAIT_GH_FUTURE
    elif state == CompleteActionState.WAIT_GH_FUTURE:
        if data.get("gh_future") is None: data["gh_future"] = data["goal_handle"].get_result_async()
        if data["gh_future"].done():
            data["node"]._goal_handles.pop(data["action_name"])
            return CompleteActionState.CHECK_STATUS
    elif state == CompleteActionState.CHECK_STATUS:
        if data["goal_handle"].status == GoalStatus.STATUS_SUCCEEDED:
            return CompleteActionState.SUCCEEDED
        elif data["goal_handle"].status == GoalStatus.STATUS_CANCELED:
            return CompleteActionState.CANCELED
        elif data["goal_handle"].status == GoalStatus.STATUS_ABORTED:
            return CompleteActionState.FAILURE
    return state


def complete_action_call(executor: Type[Executor], future):
    """Sends single action call."""
    data = {
        "future": future,
        "state": CompleteActionState.WAIT_GH,
    }
    while True:
        executor.spin_once()
        if "node" in data: data["node"].get_logger().info(f"Completing action call...", once=True)  # FIXME: this prevents a lockup issue for sweep search shell call, not sure why. it should be removed
        state = gh_state_machine(data)
        if state == CompleteActionState.SUCCEEDED:
            data["node"].get_logger().info(f"Succeeded at `{data['action_name']}`")
            return True
        elif state == CompleteActionState.FAILURE:
            data["node"].get_logger().info(f"Failed at `{data['action_name']}` with result {data['goal_handle'].status}...")
            return False
        elif state == CompleteActionState.CANCELED:
            data["node"].get_logger().info(f"Cancelled `{data['action_name']}`")
            return True
        data["state"] = state


def check_futures_done(futures: List[Future]):
    """Checks that list of futures are all done."""
    for future in futures:
        if not future.done():
            return False
    return True


def check_goal_handles_canceled(goal_handles: List):
    """Checks list of goal handles for success."""
    for goal_handle in goal_handles:
        if goal_handle.status != GoalStatus.STATUS_CANCELED:
            return False
    return True


def get_parameter_value_msg_from_type(type_, value):
    """Converts a type enum and value to its corresponding `ParameterValue` msg."""
    param_msg = ParameterValue(type=type_)
    if Parameter.Type.BOOL.value == type_:
        param_msg.bool_value = bool(value)
    elif Parameter.Type.INTEGER.value == type_:
        param_msg.integer_value = int(value)
    elif Parameter.Type.DOUBLE.value == type_:
        param_msg.double_value = float(value)
    elif Parameter.Type.STRING.value == type_:
        param_msg.string_value = str(value)
    elif Parameter.Type.BYTE_ARRAY.value == type_:
        param_msg.byte_array_value = value
    elif Parameter.Type.BOOL_ARRAY.value == type_:
        param_msg.bool_array_value = value
    elif Parameter.Type.INTEGER_ARRAY.value == type_:
        param_msg.integer_array_value = value
    elif Parameter.Type.DOUBLE_ARRAY.value == type_:
        param_msg.double_array_value = value
    elif Parameter.Type.STRING_ARRAY.value == type_:
        param_msg.string_array_value = value
    return param_msg


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
