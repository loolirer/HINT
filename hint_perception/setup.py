import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hint_perception"


def package_data_files(directory):
    """Install every file under ``directory`` to share/, preserving the tree."""
    entries = []
    for path in glob(os.path.join(directory, "**", "*"), recursive=True):
        if os.path.isfile(path):
            install_dir = os.path.join("share", package_name, os.path.dirname(path))
            entries.append((install_dir, [path]))
    return entries


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ] + package_data_files("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loolirer",
    maintainer_email="lorenzo.oliveira@ee.ufcg.edu.br",
    description="HINT perception: semantic ground segmentation (ground_segmenter) whose "
                "non-ground cells become an obstacle PointCloud2 (/ground/obstacles) for "
                "the mapless Nav2 local costmap.",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "ground_segmenter = hint_perception.ground_segmenter:main",
        ],
    },
)
