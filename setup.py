from setuptools import setup
from glob import glob

package_name = "parameter_builder"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name + "/launch",
            ["example/parameter_builder_example.launch.py"],
        ),
        ("share/" + package_name + "/config", glob("example/config/*")),
        ("share/" + package_name, ["package.xml"]),
        ("lib/" + package_name, ["example/example_node.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jafar Abdi",
    maintainer_email="jafar@picknik.ai",
    description="Python library for loading parameters in launch files",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
