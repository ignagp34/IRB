from setuptools import find_packages, setup

package_name = "irb120pe_cognitive"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/cognitive.yaml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/cognitive_arrangement_demo.launch.py",
                "launch/cognitive_demo.launch.py",
                "launch/cognitive_perception.launch.py",
                "launch/planning_scene.launch.py",
                "launch/reasoning.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cognitive IRB-120 Team",
    maintainer_email="team@example.com",
    description="Cognitive LangChain reasoning layer for the IRB-120 sorting demo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "action_adapter_node = irb120pe_cognitive.action_adapter_node:main",
            "langchain_reasoning_node = irb120pe_cognitive.langchain_reasoning_node:main",
            "perception_node = irb120pe_cognitive.perception_node:main",
            "planning_scene_sync_node = irb120pe_cognitive.planning_scene_sync_node:main",
            "gazebo_cube_helper = irb120pe_cognitive.gazebo_cube_helper:main",
            "live_cube_validator = irb120pe_cognitive.live_cube_validator:main",
            "motion_readiness_validator = irb120pe_cognitive.motion_readiness_validator:main",
            "moveit_motion_probe = irb120pe_cognitive.moveit_motion_probe:main",
            "arrangement_demo = irb120pe_cognitive.arrangement_demo:main",
            "arrangement_e2e_validator = irb120pe_cognitive.arrangement_e2e_validator:main",
        ],
    },
)
