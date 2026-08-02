import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hint_vlm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "prompts"), glob("prompts/*.txt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loolirer",
    maintainer_email="lorenzo.oliveira@ee.ufcg.edu.br",
    description="VLM-powered nodes for HINT (Gemini Robotics-ER)",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "path_planner = hint_vlm.gemini.path_planner:main",
            "visual_reasoner = hint_vlm.gemini.visual_reasoner:main",
        ],
    },
)
