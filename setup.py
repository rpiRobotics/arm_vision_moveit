from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['arm_vision_moveit'],
    package_dir={'': 'src'},
)

setup(**d)
