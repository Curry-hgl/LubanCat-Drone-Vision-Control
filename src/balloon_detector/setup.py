from setuptools import find_packages, setup


package_name = 'balloon_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = balloon_detector.detector_node:main',
            'rknn_detector_node = balloon_detector.rknn_detector_node:main',
            'cv_detector_node = balloon_detector.cv_detector_node:main',
            'hsv_detector_node = balloon_detector.hsv_detector_node:main',
        ],
    },
)
