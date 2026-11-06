from setuptools import setup
setup(name='swarm_coordination', version='0.1.0', packages=['swarm_coordination'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/swarm_coordination']),
                  ('share/swarm_coordination', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Multi Robot maintainers', maintainer_email='maintainer@example.com',
      description='Kuramoto Lissajous patrol', license='MIT',
      entry_points={'console_scripts': ['agent = swarm_coordination.agent:main']})
