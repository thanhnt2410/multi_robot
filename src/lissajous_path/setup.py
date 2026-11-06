from setuptools import setup
setup(name='lissajous_path', version='0.1.0', packages=['lissajous_path'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/lissajous_path']),
                  ('share/lissajous_path', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Multi Robot maintainers', maintainer_email='maintainer@example.com',
      description='Kuramoto Lissajous patrol', license='MIT',
      entry_points={'console_scripts': []})
