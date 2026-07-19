from setuptools import setup

package_name = "wheel_legged_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="wheel_legged",
    maintainer_email="dev@local",
    description="USB bridge NUC <-> H7",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "h7_bridge = wheel_legged_bridge.h7_bridge_node:main",
        ],
    },
)
