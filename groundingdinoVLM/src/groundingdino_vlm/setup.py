from glob import glob
from setuptools import find_packages, setup


package_name = "groundingdino_vlm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Scout Mini Maintainer",
    maintainer_email="maintainer@example.com",
    description="Provider-neutral target verification and public mock node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_verifier = groundingdino_vlm.node:main",
            "verify_target_action_server = groundingdino_vlm.action_server:main",
        ],
    },
)
