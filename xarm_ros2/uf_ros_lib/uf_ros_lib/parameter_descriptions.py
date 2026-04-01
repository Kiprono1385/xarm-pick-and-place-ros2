
#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.cub@gmail.com>
# Modified by Brian Kiprono
# Changes: Added custom pick-and-place logic, updated motion planning, integrated ROS2 nodes
 
import yaml
from launch_ros.parameter_descriptions import ParameterValue
from launch.utilities.type_utils import perform_typed_substitution

class YamlParameterValue(ParameterValue):
    def __str__(self):
        return 'uf_ros_lib.parameter_descriptions.YamlParameterValue(value={}, value_type={})'.format(self.value, self.value_type)

    def evaluate(self, context):
        """Evaluate and return parameter rule."""
        self.__evaluated_parameter_value = perform_typed_substitution(
            context, self.value, self.value_type)
        self.__evaluated_parameter_value = yaml.safe_load(self.__evaluated_parameter_value)
        return self.__evaluated_parameter_value