from setuptools import setup


setup(
    name="semantic_navigation_adapters",
    version="0.1.0",
    packages=["semantic_navigation_adapters"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/semantic_navigation_adapters"],
        ),
        ("share/semantic_navigation_adapters", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Reusable Nav2 adapters for semantic navigation",
    license="Apache-2.0",
)
