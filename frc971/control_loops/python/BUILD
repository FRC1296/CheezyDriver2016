package(default_visibility = ['//frc971:__subpackages__'])

py_library(
  name = 'controls',
  srcs = [
    'control_loop.py',
    'controls.py',
    'polytope.py',
    'libcdd.py',
  ],
  deps = [
    '//external:python-glog',
    '//external:slycot',
  ],
  data = [
    '//third_party/cddlib:_cddlib.so',
  ],
)


py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_binary(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_library(
  name = 'polydrivetrain_lib',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

genrule(
 name = 'genrule_polydrivetrain',
 visibility = ['//visibility:private'],
 cmd = '$(location //frc971/control_loops/python:polydrivetrain) $(OUTS)',
 tools = [ 
   '//frc971/control_loops/python:polydrivetrain',
 ],  
 outs = [ 
   'polydrivetrain_dog_motor_plant.h',
   'polydrivetrain_dog_motor_plant.cc',
   'polydrivetrain_cim_plant.h',
   'polydrivetrain_cim_plant.cc',
 ],  
)

