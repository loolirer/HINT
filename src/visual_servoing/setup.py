from setuptools import find_packages, setup

package_name = "visual_servoing"

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
    maintainer="loolirer",
    maintainer_email="lorenzo.oliveira@ee.ufcg.edu.br",
    description="Servo controllers: IBVS approach (visual_servo) and pure-pursuit waypoint following (pursuit_servo).",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "visual_servo = visual_servoing.visual_servo:main",
            "pursuit_servo = visual_servoing.pursuit_servo:main",
        ],
    },
)
