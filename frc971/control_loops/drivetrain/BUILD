package(default_visibility = ['//visibility:public'])

#load('/aos/build/queues', 'queue_library')

genrule(
  name = 'genrule_drivetrain',
  visibility = ['//visibility:private'],
  cmd = '$(location //frc971/control_loops/python:drivetrain) $(OUTS)',
  tools = [
    '//frc971/control_loops/python:drivetrain',
  ],
  outs = [
    'drivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.h',
    'kalman_drivetrain_motor_plant.cc',
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



#cc_binary(
#  name = 'replay_drivetrain',
#  srcs = [
#    'replay_drivetrain.cc',
#  ],
#  deps = [
#    ':drivetrain_queue',
#    '//aos/common/controls:replay_control_loop',
#    '//aos/linux_code:init',
#    '//frc971/queues:gyro',
#  ],
#)

#queue_library(
#  name = 'drivetrain_queue',
#  srcs = [
#    'drivetrain.q',
#  ],
#  deps = [
#    '//aos/common/controls:control_loop_queues',
#  ],
#)

cc_library(
  name = 'drivetrain_config',
  hdrs = [
    'drivetrain_config.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
    '//frc971:shifter_hall_effect',
  ],
)

cc_library(
  name = 'ssdrivetrain',
  srcs = [
    'ssdrivetrain.cc',
  ],
  hdrs = [
    'ssdrivetrain.h',
  ],
  deps = [
#   ':drivetrain_queue',
    ':drivetrain_config',
    '//aos/common/controls:polytope',
    '//aos/common:math',
#    '//aos/common/messages:robot_state',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
#    '//frc971:shifter_hall_effect',
#    '//aos/common/util:log_interval',
#    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_library(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.cc',
  ],
  hdrs = [
    'polydrivetrain.h',
  ],
  deps = [
#    ':drivetrain_queue',
    ':drivetrain_config',
    '//aos/common/controls:polytope',
    '//aos/common:math',
#    '//aos/common/messages:robot_state',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
#    '//aos/common/util:log_interval',
#    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_library(
  name = 'drivetrain_lib',
  srcs = [
    'drivetrain.cc',
    'frc1296.cc',
    'polydrivetrain_cim_plant.cc',
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.cc',
  ],
  hdrs = [
    'drivetrain.h',
    'polydrivetrain_cim_plant.h',
    'polydrivetrain_dog_motor_plant.h',
    'kalman_drivetrain_motor_plant.h',
    'drivetrain_dog_motor_plant.h',
  ],
  deps = [
#    ':drivetrain_queue',
    ':polydrivetrain',
    ':ssdrivetrain',
    '//aos/common/controls:control_loop',
    '//aos/common:type_traits', 
#    '//frc971/queues:gyro',
#    '//aos/common/util:log_interval',
#    '//aos/common/logging:queue_logging',
#    '//aos/common/logging:matrix_logging',
  ],
)

#cc_test(
#  name = 'drivetrain_lib_test',
#  srcs = [
#    'drivetrain_lib_test.cc',
#  ],
#  deps = [
#    '//aos/testing:googletest',
#    ':drivetrain_queue',
#    ':drivetrain_lib',
#    ':drivetrain_config',
#    '//aos/common/controls:control_loop_test',
#    '//frc971/control_loops:state_feedback_loop',
#    '//frc971/queues:gyro',
#    '//aos/common:queues',
#    '//y2014:constants',
#    '//y2014/control_loops/drivetrain:polydrivetrain_plants',
#  ],
#)
