from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
#packages=['sb_jaus'],
#package_dir={''}
)

setup(**d)
