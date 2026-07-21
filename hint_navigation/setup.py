import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hint_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loolirer",
    maintainer_email="lorenzo.oliveira@ee.ufcg.edu.br",
    description="Nav2 reactive-navigation integration for HINT: a trajectory_navigator "
                "adapter that grounds VLM markers into a nav_msgs/Path and drives Nav2's "
                "FollowPath (MPPI) controller over a mapless local costmap (fed by "
                "hint_perception's ground_segmenter).",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "trajectory_navigator = hint_navigation.trajectory_navigator:main",
        ],
    },
)
