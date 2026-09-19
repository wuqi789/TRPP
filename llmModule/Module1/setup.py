from setuptools import setup


setup(
    name="llm_module1",
    version="0.1.0",
    packages=["llm_agent"],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="LLM semantic reasoner for navigation intents",
    license="Apache-2.0",
    entry_points={"console_scripts": ["llm_agent_node = llm_agent.llm_agent_node:main"]},
)

