from rclpy.node import Node
from collections import namedtuple

# https://stackoverflow.com/a/58950681
class defDictToObject(object):
    def __init__(self, myDict):
        for key, value in myDict.items():
            if type(value) == dict:
                setattr(self, key, defDictToObject(value))
            else:
                setattr(self, key, value)


class EzNode(Node):

    def declare_parameter_ez(self, name, value):
        if not self.has_parameter(name): self.declare_parameter(name, value)

    def get_param_values_dict_by_prefix(self, prefix):
        raise NotImplementedError
        # def convert_param_dict(pd):
        #     out = {}
        #     for k,v in pd.items():
        #         ksplit = k.split(".")
        #         for n in ksplit:
        #             out
        # p = self.get_parameters_by_prefix(prefix)
        # {k: v.value for k, v in posctl.items()}
        # return 

    def get_param_values_obj_by_prefix(self, prefix):
        raise NotImplementedError
        return self.get_parameters_dict_by_prefix(prefix)
    
    @property
    def seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9
    
    @property
    def nanoseconds(self):
        return self.get_clock().now().nanoseconds
