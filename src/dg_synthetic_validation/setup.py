from glob import glob
import os
from setuptools import setup


package_name = "dg_synthetic_validation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    zip_safe=True,
    maintainer="liaojingwu20041031",
    maintainer_email="206929594+liaojingwu20041031@users.noreply.github.com",
    description="Synthetic ROS2 validation for DG-202611 navigation health paths.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "synthetic_injector_node = dg_synthetic_validation.synthetic_injector_node:main",
            "synthetic_evaluator_node = dg_synthetic_validation.evaluator_node:main",
            "run_scenario = dg_synthetic_validation.scenario_runner:main",
            "run_all_s01_s04 = dg_synthetic_validation.scenario_runner:run_all_main",
            "monitor_scenario = dg_synthetic_validation.monitor_node:main",
            "visualization_markers_node = dg_synthetic_validation.visualization_markers_node:main",
            "capture_evidence = dg_synthetic_validation.evidence_capture:main",
            "write_evidence_manifest = dg_synthetic_validation.evidence_manifest:main",
        ],
    },
)
