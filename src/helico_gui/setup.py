import os
from glob import glob
from setuptools import setup

package_name = "helico_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
    ],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Naythan Aung",
    maintainer_email="naythanaung@hotmail.com",
    description="Helico Meca500 workbench GUI",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "helico_gui = helico_gui.main:main",
        ],
    },
)
