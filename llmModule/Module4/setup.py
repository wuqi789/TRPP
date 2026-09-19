from setuptools import setup


setup(
    name="llm_module4",
    version="0.1.0",
    packages=[
        "execution_core",
        "route_planning",
        "module4_ros_interface",
    ],
    install_requires=["setuptools", "PyYAML", "Pillow"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="VLM route bridge from verified semantic goals to Scout NeuPAN",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "navigation_executor_node = module4_ros_interface.navigation_executor_node:main"
        ]
    },
)
