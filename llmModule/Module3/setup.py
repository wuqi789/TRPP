from setuptools import setup


setup(
    name="llm_module3",
    version="0.1.0",
    packages=[
        "verification_core",
        "entity_checker",
        "topology_checker",
        "geometry_checker",
        "dynamic_checker",
        "module3_ros_interface",
    ],
    install_requires=["setuptools", "networkx", "PyYAML"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Multi-level verification for semantic navigation goals",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "verification_node = module3_ros_interface.verification_node:main"
        ]
    },
)
