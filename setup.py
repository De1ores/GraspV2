from setuptools import setup


package_name = "graspv2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/config",
            ["config/x2_aimdk_hardware.json"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ruiujia",
    maintainer_email="ruiujia@example.com",
    description="Official-IK X2 planning and visually verified OmniPicker grasping",
    license="MIT",
    entry_points={
        "console_scripts": [
            "x2_aimdk_hardware = graspv2.aimdk_hardware:main",
            "x2_mc_custom_grasp = graspv2.mc_custom_grasp:main",
        ],
    },
)
