from glob import glob

from setuptools import find_packages, setup

package_name = "nxp_cup_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ihtisham",
    maintainer_email="ihtishamarif855@gmail.com",
    description="Camera line follower for the NXP Cup car",
    license="MIT",
    entry_points={
        "console_scripts": [
            "line_follower = nxp_cup_control.line_follower:main",
        ],
    },
)
