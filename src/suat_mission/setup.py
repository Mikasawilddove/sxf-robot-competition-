from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['suat_mission', 'suat_mission.navigation', 'suat_mission.vision', 'suat_mission.manipulation', 'suat_mission.automation'], package_dir={'': 'src'}))
