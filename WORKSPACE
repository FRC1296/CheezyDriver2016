new_local_repository(
  name = 'usr_repo',
  path = '/usr',
  build_file = 'debian/usr.BUILD',
)

new_git_repository(
  name = 'slycot_repo',
  remote = 'https://github.com/avventi/Slycot.git',
  build_file = 'debian/slycot.BUILD',
  commit = '5af5f283cb23cbe23c4dfea4d5e56071bdbd6e70',
)

bind(
  name = 'slycot',
  actual = '@slycot_repo//:slycot',
)

new_http_archive(
  name = 'arm_frc_linux_gnueabi_repo',
  build_file = 'tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD',
  sha256 = '9e93b0712e90d11895444f720f0c90c649fb9becb4ca28bb50749d9074eb1306',
  url = 'http://frc971.org/Build-Dependencies/roborio-compiler-2016.tar.gz',
)

# Recompressed version of the one downloaded from Linaro at
# <https://releases.linaro.org/15.05/components/toolchain/binaries/arm-linux-gnueabihf/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.xz>,
# with workarounds for <https://github.com/bazelbuild/bazel/issues/574> and the
# top-level folder stripped off.
new_http_archive(
  name = 'linaro_linux_gcc_4.9_repo',
  build_file = 'compilers/linaro_linux_gcc_4.9.BUILD',
  sha256 = '25e97bcb0af4fd7cd626d5bb1b303c7d2cb13acf2474e335e3d431d1a53fbb52',
  url = 'http://frc971.org/Build-Dependencies/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.gz',
)

new_git_repository(
  name = 'python_gflags_repo',
  remote = 'https://github.com/gflags/python-gflags.git',
  build_file = 'debian/gflags.BUILD',
  commit = '41c4571864f0db5823e07715317e7388e94faabc',
)

bind(
  name = 'python-gflags',
  actual = '@python_gflags_repo//:gflags',
)

new_http_archive(
  name = 'python_glog_repo',
  build_file = 'debian/glog.BUILD',
  sha256 = '953fd80122c48023d1148e6d1bda2763fcab59c8a81682bb298238a5935547b0',
  url = 'https://pypi.python.org/packages/source/g/glog/glog-0.1.tar.gz',
  strip_prefix = 'glog-0.1',
)

bind(
  name = 'python-glog',
  actual = '@python_glog_repo//:glog',
)

new_http_archive(
  name = 'allwpilib_ni_libraries_repo',
  build_file = 'debian/ni-libraries.BUILD',
  sha256 = '821687afbee2d7531fb3e47d8d58ac10005695e59685be3ac3aa00b3179faf52',
  url = 'http://frc971.org/Build-Dependencies/allwpilib_ni-libraries_20749ed.tar.gz',
  strip_prefix = 'ni-libraries',
)

bind(
  name = 'ni-libraries',
  actual = '@allwpilib_ni_libraries_repo//:ni-libraries',
)
