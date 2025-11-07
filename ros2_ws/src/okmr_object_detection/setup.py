from setuptools import find_packages, setup
import os
from glob import glob

package_name = "okmr_object_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "models"),
            glob(os.path.join("models", "*.onnx")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eryk",
    maintainer_email="erykhalicki0@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "example_detector = okmr_object_detection.example_detector:main",
            "color_detector = okmr_object_detection.color_detector:main",
            "lid_detector = okmr_object_detection.lid_detector:main",
            "onnx_segmentation_detector = okmr_object_detection.onnx_segmentation_detector:main",
            "mask_offset_node = okmr_object_detection.mask_offset_node:main",
            "mask_to_bbox = okmr_object_detection.mask_to_bbox:main",
        ],
    },
)
