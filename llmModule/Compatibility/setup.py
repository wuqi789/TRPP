from setuptools import setup


setup(
    name="semantic_navigation_compatibility",
    version="0.1.0",
    packages=["semantic_navigation_compatibility"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/semantic_navigation_compatibility"],
        ),
        ("share/semantic_navigation_compatibility", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Legacy topic adapters for semantic navigation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "compatibility_node = semantic_navigation_compatibility.node:main",
        ],
    },
)
