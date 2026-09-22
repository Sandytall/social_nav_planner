from setuptools import setup

package_name = "social_nav_benchmarks"

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
    maintainer="Sandeep",
    maintainer_email="kaustubh@origin.tech",
    description="SocialNav benchmarking framework (MASTER_PROMPT §29/§38/§39).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "social-nav-benchmark = social_nav_benchmarks.runner:main",
        ],
    },
)
