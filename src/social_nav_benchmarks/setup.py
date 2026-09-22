from glob import glob

from setuptools import setup

package_name = "social_nav_benchmarks"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Baseline planner profiles selected by the runner via SOCIAL_NAV_PARAMS.
        ("share/" + package_name + "/config/planners", glob("config/planners/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sandeep",
    maintainer_email="sandeepnaik89711@gmail.com",
    description="Automated benchmarking framework for the SocialNav Planner.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "social-nav-benchmark = social_nav_benchmarks.runner:main",
            "social-nav-analyze = social_nav_benchmarks.analyze:main",
            "social-nav-report = social_nav_benchmarks.report_html:main",
        ],
    },
)
