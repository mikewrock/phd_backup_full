## *********************************************************
## 
## File autogenerated for the phd package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 235, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'GROUP_ONE', 'lower': 'group_one', 'srcline': 109, 'name': 'Group_One', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::GROUP_ONE', 'field': 'DEFAULT::group_one', 'state': True, 'parentclass': 'DEFAULT', 'groups': [{'upper': 'GROUP2', 'lower': 'group2', 'srcline': 109, 'name': 'GROUP2', 'parent': 1, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Group_One', 'class': 'DEFAULT::GROUP_ONE::GROUP2', 'field': 'DEFAULT::GROUP_ONE::group2', 'state': True, 'parentclass': 'DEFAULT::GROUP_ONE', 'groups': [], 'parameters': [{'srcline': 17, 'description': 'A third level group parameter', 'max': 'std::numeric_limits<double>::infinity()', 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/mike/catkin_ws/src/phd/cfg/param_config.cfg', 'name': 'group2_double', 'edit_method': '', 'default': 3.333, 'level': 0, 'min': '-std::numeric_limits<double>::infinity()', 'type': 'double'}], 'type': '', 'id': 2}], 'parameters': [{'srcline': 15, 'description': 'A second level group parameter', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/mike/catkin_ws/src/phd/cfg/param_config.cfg', 'name': 'group1_int', 'edit_method': '', 'default': 2, 'level': 1, 'min': -2147483648, 'type': 'int'}], 'type': 'collapse', 'id': 1}], 'parameters': [{'srcline': 280, 'description': 'Cloud Save Filename', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'cloud_name', 'edit_method': '', 'default': 'cloud', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 280, 'description': 'Cloud Alignment Number', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'cloud_number', 'edit_method': '', 'default': 1, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 280, 'description': 'Marker Intensiy Cutoff', 'max': 3000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'intensity_min', 'edit_method': '', 'default': 900, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 280, 'description': 'Marker Intensiy Cutoff', 'max': 3000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'intensity_max', 'edit_method': '', 'default': 1200, 'level': 0, 'min': 0, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

