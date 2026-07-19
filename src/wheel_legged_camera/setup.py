from setuptools import setup

package_name = "wheel_legged_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "opencv-python"],
    zip_safe=True,
    maintainer="wheel_legged",
    maintainer_email="dev@local",
    description="NUC camera node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_node = wheel_legged_camera.camera_node:main",
            "mjpeg_server = wheel_legged_camera.mjpeg_server:main",
        ],
    },
)
