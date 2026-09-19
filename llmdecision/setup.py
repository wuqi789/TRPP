from glob import glob
from setuptools import find_packages, setup

setup(
    name="llmdecision",
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    py_modules=["main"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/llmdecision"]),
        ("share/llmdecision", ["package.xml"]),
        ("share/llmdecision/config", glob("config/*.yaml")),
        ("share/llmdecision/examples", glob("examples/input_example.json")),
    ],
    install_requires=["setuptools", "PyYAML", "Pillow"],
    zip_safe=True,
    maintainer="Scout Mini Maintainer",
    maintainer_email="maintainer@example.com",
    description="Pushability contracts with mock and external-provider adapters",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pushability_action_server = llmdecision_ros.action_server:main",
        ]
    },
)
