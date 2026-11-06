from setuptools import setup
setup(name='patrol_evaluation', version='0.1.0', packages=['patrol_evaluation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/patrol_evaluation']),
                  ('share/patrol_evaluation', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Multi Robot maintainers', maintainer_email='maintainer@example.com',
      description='Kuramoto Lissajous patrol', license='MIT',
      entry_points={'console_scripts': ['evaluate = patrol_evaluation.node:main', 'offline = patrol_evaluation.offline:main']})
