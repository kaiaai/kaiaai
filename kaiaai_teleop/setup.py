from setuptools import setup

package_name = 'kaiaai_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ilia O.',
    author_email='iliao@remake.ai',
    maintainer='Ilia O.',
    maintainer_email='iliao@remake.ai',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=('Control Kaia.ai-compatible robots using a keyboard'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = kaiaai_teleop.teleop_keyboard:main'
        ],
    },
)
