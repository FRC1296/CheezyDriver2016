# The primary client logging interface.
cc_library(
  name = 'logging',
  visibility = ['//visibility:public'],
  hdrs = [
    'logging.h',
    'interface.h',
    'context.h',
  ],
  srcs = [
  ],
  deps = [
    '//aos/common/libc:aos_strerror',
    '//aos/common:macros',
  ],
)

cc_library(
  name = 'matrix_logging',
  visibility = ['//visibility:public'],
  srcs = [
  ],
  hdrs = [
    'matrix_logging.h',
  ],
  deps = [
    ':logging',
    '//third_party/eigen',
  ],
)


