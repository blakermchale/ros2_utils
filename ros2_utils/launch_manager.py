#!/usr/bin/env python3
from pathlib import Path
from launch import LaunchDescription, Action, LaunchContext
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import TextSubstitution


from typing import List
import ast
import argparse
import yaml


class LaunchManager(LaunchDescription):
    
    def add_arg(self, name, default_value=None, description=None, choices=None, **kwargs):
        default_value = str(default_value) if default_value is not None else None
        launch_configuration = LaunchConfiguration(name, default=default_value)
        setattr(self, name, launch_configuration)
        declare_action = DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
            choices=choices,
            **kwargs
        )
        self.add_action(declare_action)
        return launch_configuration

    def print_info(self, msg):
        log = LogInfo(msg=msg)
        self.add_action(log)
        return log
    
    def add_opaque_function(self, function, yaml_file="", **kwargs):
        def helper(context: LaunchContext):
            # Read launch args from YAML
            yaml_args = {}
            if yaml_file != "":
                param_file = ParameterFile(
                    param_file=yaml_file,
                    allow_substs=True)
                param_file_path = param_file.evaluate(context)
                with open(param_file_path, 'r') as f:
                    yaml_args = yaml.load(f, Loader=yaml.FullLoader)
            # Create contextualized args from YAML and passed launch args
            eval_args = argparse.Namespace()
            launch_args = self.get_launch_arguments()
            for arg in launch_args:
                # Prioritize launch args in YAML
                if arg.name in yaml_args:
                    value = yaml_args[arg.name]
                else:  # Read from passed launch args instead
                    value = LaunchConfiguration(arg.name).perform(context)
                    try:
                        value = ast.literal_eval(value)
                    except:
                        if value.lower() in ["nan", "inf", "infinity"]:
                            value = float(value)
                setattr(eval_args, arg.name, value)
            setattr(eval_args, "context", context)
            return function(eval_args)
        opaque_function = OpaqueFunction(function=helper, **kwargs)
        self.add_action(opaque_function)
        return opaque_function

    def add_include_launch_description(self, package:str, launch_file:str, launch_arguments={}, **kwargs) -> IncludeLaunchDescription:
        """Adds IncludeLaunchDescription to launch manager. Simplifies launch include process and allows for propagation of arguments.

        Args:
            package (str): _description_
            launch_file (str): _description_
            launch_arguments (dict, optional): Launch arguments to pass forward. Defaults to {}.
            forward_arguments (bool, optional): Get arguments from passed launch file and add to current with namespace. Defaults to False.
            namespace (str, optional): _description_. Defaults to "".
        """
        path = get_launch_file(package, launch_file)
        launch_description_source = PythonLaunchDescriptionSource(str(path))
        # Launch args must be strings
        str_launch_args = {}
        for k, v in launch_arguments.items():
            str_launch_args[k] = str(v)
        # Add launch description
        include_action = IncludeLaunchDescription(
            launch_description_source,
            launch_arguments=str_launch_args.items(),
            **kwargs
        )
        self.add_action(include_action)
        return include_action
    
    def add_include_launch_args(self, package: str, launch_file: str, exclude=[]) -> List[DeclareLaunchArgument]:
        """Adds arguments from included launch file.
        
        This is separated from add_include_launch_description since args cannot be passed forward when in OpaqueFunction.
        There needs to be a way to forward arguments so provide a method of getting arguments without include.
        """
        ld = get_launch_description(package, launch_file)
        launch_args = ld.get_launch_arguments()
        for arg in launch_args:
            default_value = arg.default_value
            description = arg.description.split("Valid choices are:", 1)[0].strip()  # appended by choices arg
            if arg.name in exclude:
                continue
            if isinstance(arg.default_value[0], TextSubstitution):
                default_value = arg.default_value[0].text
            self.add_arg(arg.name, default_value, description, arg.choices, condition=arg.condition)
        return launch_args
    
    def add_launch_description(self, ld: LaunchDescription):
        actions = ld.describe_sub_entities()
        for action in actions:
            self.add_action(action)
    
    def add_action_list(self, actions: List[Action]):
        for action in actions:
            self.add_action(action)


def get_launch_description(package: str, launch_file: str) -> LaunchDescription:
    path = get_launch_file(package, launch_file)
    launch_description_source = PythonLaunchDescriptionSource(str(path))
    ld = launch_description_source.try_get_launch_description_without_context()
    return ld


def get_launch_file(package: str, launch_file: str) -> Path:
    path = Path(get_package_share_directory(package))
    launch_files = list(path.rglob(launch_file))
    num_launches = len(launch_files)
    if num_launches != 1:
        raise ValueError(f"Found {num_launches} multiple matching launch files in {package}. Expected 1.")
    return launch_files[0]


def get_local_arguments(args: argparse.Namespace, context, yaml_file: str=""):
    """Stores launch arguments in dictionary using RCL context."""
    if yaml_file != "":
        param_file = ParameterFile(
            param_file=yaml_file,
            allow_substs=True)
        param_file_path = param_file.evaluate(context)
        with open(param_file_path, 'r') as f:
            config_vals = yaml.load(f, Loader=yaml.FullLoader)
        # Overrides passed launch args with config files
        for k, v in config_vals.items():
            setattr(args, k, v)
    return args