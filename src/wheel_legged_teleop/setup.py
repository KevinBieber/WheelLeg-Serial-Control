from setuptools import setup

package_name = "wheel_legged_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheel_legged",
    maintainer_email="dev@local",
    description="PC command/status TCP gateway",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "net_gateway = wheel_legged_teleop.net_gateway_node:main",
        ],
    },
)
