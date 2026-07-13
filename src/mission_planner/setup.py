import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mission_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "missions"), glob("missions/*.yaml")),
        (os.path.join("share", package_name, "templates"), glob("templates/*.txt")),
        (os.path.join("share", package_name, "config"), glob("config/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loolirer",
    maintainer_email="lorenzo.oliveira@ee.ufcg.edu.br",
    description="Semantic mission planner for HINT",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mission_planner_node = mission_planner.mission_planner_node:main",
        ],
    },
)
