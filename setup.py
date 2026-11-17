# Used by catkin (catkin_python_setup in CMakeLists.txt). Without ROS: pip install -e .
try:
    from catkin_pkg.python_setup import generate_distutils_setup
    from setuptools import setup
    setup(**generate_distutils_setup(packages=["drg"], package_dir={"": "src"}))
except ImportError:
    from setuptools import setup
    setup(name="drg", version="1.0.0", packages=["drg"], package_dir={"": "src"},
          install_requires=["numpy", "scipy", "pyyaml", "open3d==0.18.0"])
