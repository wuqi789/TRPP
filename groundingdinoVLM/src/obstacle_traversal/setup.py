from glob import glob
from setuptools import find_packages, setup

setup(
    name="obstacle_traversal",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/obstacle_traversal"]),
        ("share/obstacle_traversal", ["package.xml"]),
        ("share/obstacle_traversal/config", glob("config/*.yaml")),
        ("share/obstacle_traversal/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Scout Mini Maintainer",
    maintainer_email="maintainer@example.com",
    description="Fail-closed obstacle traversal manager, scan filter, and velocity gate",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "traversal_manager = obstacle_traversal.manager:main",
        "scan_filter = obstacle_traversal.scan_filter:main",
        "velocity_gate = obstacle_traversal.velocity_gate:main",
        "approach_controller = obstacle_traversal.approach_controller:main",
        "system_diagnostics = obstacle_traversal.system_diagnostics:main",
        "core_acceptance_driver = obstacle_traversal.core_acceptance_driver:main",
        "publish_instruction = obstacle_traversal.publish_instruction:main",
        "preflight = obstacle_traversal.preflight:main",
    ]},
)
