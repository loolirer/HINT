import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gemini_robotics_er"

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
    description="Gemini Robotics-ER nodes for HINT",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "description_detector = gemini_robotics_er.description_detector:main",
            "visual_question = gemini_robotics_er.visual_question:main",
            "trajectory_generator = gemini_robotics_er.trajectory_generator:main",
            "visual_reasoner = gemini_robotics_er.visual_reasoner:main",
        ],
    },
)
