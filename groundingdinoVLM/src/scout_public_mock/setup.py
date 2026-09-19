from setuptools import find_packages, setup

setup(
    name="scout_public_mock",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/scout_public_mock"]),
        ("share/scout_public_mock", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Scout Mini Maintainer",
    maintainer_email="maintainer@example.com",
    description="Offline mock providers and fixed route driver for the public demo",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "mock_verify_target = scout_public_mock.mock_nodes:verify_target_main",
        "mock_assess_pushability = scout_public_mock.mock_nodes:assess_pushability_main",
        "mock_probe_pushability = scout_public_mock.mock_nodes:probe_pushability_main",
        "mock_approach_obstacle = scout_public_mock.mock_nodes:approach_obstacle_main",
        "mock_yolo_detector = scout_public_mock.mock_nodes:yolo_main",
        "mock_piper_arm = scout_public_mock.mock_nodes:arm_main",
        "fixed_route_driver = scout_public_mock.mock_nodes:route_main",
    ]},
)
