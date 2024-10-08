from setuptools import find_packages, setup

package_name = 'vision_module'

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
    maintainer='jaqq',
    maintainer_email='chen.jiunhan@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_module.vision_node:main',
            'object_detection_node = vision_module.object_detection_node:main',
            'semantic_segmentation_node = vision_module.semantic_segmentation_node:main'
        ],
    },
)
