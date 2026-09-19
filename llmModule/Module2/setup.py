from setuptools import setup


setup(
    name="llm_module2",
    version="0.1.0",
    packages=[
        "semantic_graph",
        "builder",
        "query",
        "providers",
        "matching",
        "resolution",
        "ros_interface",
    ],
    install_requires=[
        "setuptools",
        "networkx",
        "PyYAML",
        "typing-extensions>=4.8,<5",
    ],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Semantic scene graph grounding and query layer",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["semantic_graph_node = ros_interface.semantic_graph_node:main"]
    },
)
