from setuptools import setup


setup(
    name="llm_semantic_affordance",
    version="0.1.0",
    packages=["llm_semantic_affordance"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/llm_semantic_affordance"],
        ),
        ("share/llm_semantic_affordance", ["package.xml"]),
        ("share/llm_semantic_affordance/config", ["config/affordance.yaml"]),
        ("share/llm_semantic_affordance/launch", ["launch/affordance.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="VLM pushability instance mapping for Scout semantic navigation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pushability_mapper = llm_semantic_affordance.node:main",
        ],
    },
)
