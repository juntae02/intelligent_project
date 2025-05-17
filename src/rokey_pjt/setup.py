from setuptools import find_packages, setup

package_name = 'rokey_pjt'

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
    maintainer='juntae02',
    maintainer_email='juntaepark02@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_depict1 = rokey_pjt.ai_depict:main',
            'capture_image = rokey_pjt.tb4_capture_image:main',
            'cont_cap_image = rokey_pjt.tb4_cont_capture_image:main',
            'det_obj = rokey_pjt.tb4_yolov8_obj_det:main',
            'det_obj_thread = rokey_pjt.tb4_yolov8_obj_det_thread:main',
            'det_obj_track = rokey_pjt.tb4_yolov8_obj_det_track:main',
            'check_depth = rokey_pjt.depth_checker:main',
            'check_depth_click = rokey_pjt.depth_checker_mouse_click:main',
            'yolo_depth_checker = rokey_pjt.3_tb4_yolo_bbox_depth_checker:main',
            'test_server = rokey_pjt.robot_action_copy:main',
            'test_cli = rokey_pjt.juntae:main',
            'tf_trans = rokey_pjt.4_tb4_tf_transform:main',
        ],
    },
)
