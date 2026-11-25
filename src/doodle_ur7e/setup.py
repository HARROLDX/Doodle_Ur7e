from setuptools import setup, find_packages
setup(name='doodle_ur7e', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/doodle_ur7e']),
                  ('share/doodle_ur7e', ['package.xml']),
                  ('share/doodle_ur7e/launch', ['launch/bringup.launch.py'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='HARROLDX', maintainer_email='maintainer@example.invalid',
      description='DoodleDroid drawing workflow adapted for UR7e and ROS 2 Humble',
      license='MIT', entry_points={'console_scripts':['doodle = doodle_ur7e.cli:main']})
