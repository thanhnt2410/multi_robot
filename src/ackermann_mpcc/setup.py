from setuptools import setup
setup(name='ackermann_mpcc', version='0.1.0', packages=['ackermann_mpcc'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/ackermann_mpcc']),
                  ('share/ackermann_mpcc', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Multi Robot maintainers', maintainer_email='maintainer@example.com',
      description='Kuramoto Lissajous patrol', license='MIT',
      entry_points={'console_scripts': ['controller = ackermann_mpcc.node:main']})
