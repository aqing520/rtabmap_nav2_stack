from setuptools import find_packages, setup

package_name = "bt468_rtk_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bt468_rtk.launch.py"]),
        ("share/" + package_name + "/config", ["config/bt468_rtk.yaml"]),
    ],
    install_requires=["setuptools", "pyserial>=3.5"],
    zip_safe=True,
    maintainer="pansen",
    maintainer_email="pansen@local",
    description="ROS 2 BT-468 RTK GNSS node and serial parsing helpers.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bt468_rtk_node = bt468_rtk_driver.ros2_node:main",
            "bt468_rtk_cli = bt468_rtk_driver.cli:main",
        ],
    },
)
