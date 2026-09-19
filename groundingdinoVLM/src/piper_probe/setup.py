from glob import glob
from setuptools import find_packages, setup

setup(
    name="piper_probe",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/piper_probe"]),
        ("share/piper_probe", ["package.xml"]),
        ("share/piper_probe/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Scout Mini Maintainer",
    maintainer_email="maintainer@example.com",
    description="Piper mechanical pushability action server for Isaac Sim",
    license="Apache-2.0",
    test_suite="test.test_piper_core",
    entry_points={"console_scripts": [
        "probe_server = piper_probe.probe_server:main",
        "yolo_obstacle_detector = piper_probe.yolo_obstacle_detector:main",
    ]},
)
