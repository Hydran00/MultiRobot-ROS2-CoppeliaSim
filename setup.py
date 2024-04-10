from setuptools import setup
import os

package_name = 'ur_coppeliasim'
share_dir = os.path.join("share", package_name)
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Davide',
    author_email='davide.nardi-1@studenti.unitn.it',
    maintainer='davide.nardi-1@studenti.unitn.it',
    maintainer_email='davide.nardi-1@studenti.unitn.it',
    keywords=['ROS 2', 'example', 'package'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    description='A short description of your package',
    long_description_content_type='text/markdown',
    license='Apache License, Version 2.0',
    data_files=[
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    (share_dir, ["package.xml"]),
    #(os.path.join(share_dir, "launch"), ["launch/endpoint.launch.py"]),
    ],
)