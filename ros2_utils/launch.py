#!/usr/bin/env python3
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
import ast
import jinja2
import xacro
import os
import yaml


def get_launch_arguments(launch_args: list):
    """Converts dictionary of launch args into list of DeclareLaunchArgument's"""
    validate_launch_args(launch_args)
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description'], choices=param.get("choices")) for param in launch_args]


def convert_type(value, atype):
    """Converts string using type identifier."""
    if atype == "int": return int(value)
    elif atype == "bool": return value == "true"
    elif atype == "float": return float(value)
    elif atype == "list": return ast.literal_eval(value)
    else: return value


def get_local_arguments(launch_args: dict, context, yaml_file: str=""):
    """Stores launch arguments in dictionary using RCL context."""
    validate_launch_args(launch_args)
    largs = {param["name"]: convert_type(LaunchConfiguration(param["name"]).perform(context), param.get("type")) for param in launch_args}
    if yaml_file != "":
        param_file = ParameterFile(
            param_file=yaml_file,
            allow_substs=True)
        param_file_path = param_file.evaluate(context)
        with open(param_file_path, 'r') as f:
            config_vals = yaml.load(f, Loader=yaml.FullLoader)
        # Overrides passed launch args with config files
        for k, v in config_vals.items():
            # TODO: should we force yaml params to be in the launch args dict
            largs[k] = v  #TODO: make sure this is the proper type
    return largs


def validate_launch_args(launch_args: list):
    """Validates launch args list does not contain any duplicate names."""
    names = []
    for arg in launch_args:
        name = arg["name"]
        if name in names:
            raise ValueError(f"Repeated name '{name}' found in launch arguments. Please make sure to remove the duplicate or add namespacing.")
        names.append(name)


def parse_model_file(file_path, mappings):
    namespace = mappings["namespace"]
    robot_desc = None
    if file_path.split('.')[-1] == "xacro":
        tmp_path = f'/tmp/{namespace}.urdf'
        doc = xacro.process_file(file_path, mappings=mappings)
        robot_desc = doc.toprettyxml(indent='  ')
        tmp_file = open(tmp_path, 'w')
        tmp_file.write(robot_desc)
        tmp_file.close()
    elif file_path.split('.')[-1] == "erb":
        tmp_path = f'/tmp/{namespace}.sdf'
        cmd = "erb"
        for (key, val) in mappings.items():
            cmd += f" {key}={val}"
        cmd += f" {file_path} > {tmp_path}"
        os.system(cmd)
    elif file_path.split('.')[-1] == "jinja":
        tmp_path = f'/tmp/{namespace}.sdf'
        templateFilePath = jinja2.FileSystemLoader(os.path.dirname(file_path))
        jinja_env = jinja2.Environment(loader=templateFilePath)
        j_template = jinja_env.get_template(os.path.basename(file_path))
        output = j_template.render(mappings)
        with open(tmp_path, 'w') as sdf_file:
            sdf_file.write(output)
    else:
        tmp_path = file_path
    return tmp_path, robot_desc

def combine_names(l: list, sep: str):
    l = list(filter(None, l))  # Remove empty strings
    return sep.join(l)
