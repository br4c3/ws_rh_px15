from setuptools import find_packages, setup

package_name = 'guidance'

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
    maintainer='rohang',
    maintainer_email='rohang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinate = guidance.tcoordinate_tf:main',
            'Transformation = guidance.Transformation:main',
            'pidg_land = guidance.tpid_autoland:main',
            'pidg_2025 = guidance.pidg_2025:main',
            'FSM = guidance.FSM:main',
            'gcs = guidance.gcs:main',
            'hold_pidg_land = guidance.pidg_hold2land:main',
            'new_pland = guidance.pibvs_autoland:main',
            'siyi_yolo = guidance.tcam_siyi_yolo:main',
            'ocam_yolo = guidance.cam_ocam_yolo:main',
            'servo = guidance.servo:main',
            'yaw_align = guidance.yaw_alignment_node:main'
            
        ],
    },
)
