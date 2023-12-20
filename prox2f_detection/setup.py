from setuptools import find_packages, setup

package_name = "prox2f_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sakai Hibiki",
    maintainer_email="sakai_hibiki@stu.kanazawa-u.ac.jp",
    description="Object detection for prox2f.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
