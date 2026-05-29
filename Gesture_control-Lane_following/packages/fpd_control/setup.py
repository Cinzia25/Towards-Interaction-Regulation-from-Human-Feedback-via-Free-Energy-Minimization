from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['fpd_control', 'fpd_control.fpd_lane_controller'],
    package_dir={'': 'src'},
)

setup(**setup_args)

