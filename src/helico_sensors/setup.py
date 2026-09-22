from setuptools import find_packages, setup

package_name = "helico_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        exclude=["test"]
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="naythan",
    maintainer_email="naythan@example.com",
    description="Helico bench sensor nodes",
    license="Apache-2.0",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            "fake_sensors = helico_sensors.fake_sensors:main",
        ],
    },
)