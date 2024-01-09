from setuptools import find_packages, setup
from generate_parameter_library_py.setup_helper import generate_parameter_module
from glob import glob


package_name = "prox2f_detection"

generate_parameter_module(
    "object_detection_params", "prox2f_detection/object_detection_params.yaml"
)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sakai Hibiki",
    maintainer_email="sakai_hibiki@stu.kanazawa-u.ac.jp",
    description="Object detection for prox2f.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detection_node = prox2f_detection.object_detection_node:main"
        ],
    },
)
