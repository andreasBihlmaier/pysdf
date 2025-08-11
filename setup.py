from setuptools import find_packages, setup

package_name = 'pysdf'

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(exclude=['test']),
  package_dir={'': '.'},
  data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='Andreas Bihlmaier',
  maintainer_email='andreas.bihlmaier@gmx.de',
  description='Python library to parse SDF into class hierarchy and export URDF',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'sdf2urdf = pysdf.sdf2urdf:main',
    ],
  },
)
