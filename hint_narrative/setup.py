import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hint_narrative"

# Missions live one-per-directory (missions/<name>/mission.yaml); install each
# preserving its subdirectory so the runtime artifacts stay grouped per mission.
mission_data = [
    (os.path.join("share", package_name, os.path.dirname(p)), [p])
    for p in glob("missions/**/*.yaml", recursive=True)
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "prompts"), glob("prompts/*.txt")),
    ] + mission_data,
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
            "narrative_navigation = hint_narrative.narrative_navigation:main",
        ],
    },
)
