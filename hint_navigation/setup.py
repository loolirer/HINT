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
    description="Nav2 reactive-navigation integration for HINT and the image<->metric "
                "bridge (sole owner of the camera rig, camera_rig.CameraRig): path_projector "
                "(VLM waypoints -> nav_msgs/Path -> Nav2 FollowPath/MPPI), obstacle_projector "
                "(ground mask -> obstacle PointCloud2), and visual_debug (composited /debug).",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "path_projector = hint_navigation.path_projector:main",
            "obstacle_projector = hint_navigation.obstacle_projector:main",
            "visual_debug = hint_navigation.visual_debug:main",
        ],
    },
)
