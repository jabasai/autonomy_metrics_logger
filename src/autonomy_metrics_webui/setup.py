from glob import glob
from setuptools import setup


package_name = "autonomy_metrics_webui"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/templates", glob("autonomy_metrics_webui/templates/*")),
        ("share/" + package_name + "/static", glob("autonomy_metrics_webui/static/*")),
    ],
    install_requires=["setuptools", "flask"],
    zip_safe=True,
    maintainer="Ibrahim",
    maintainer_email="ibrahim.hroub7@gmail.com",
    description="Industrial-style web UI for autonomy metrics and robustness monitoring.",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "autonomy_metrics_webui_server = autonomy_metrics_webui.webui_server:main",
        ],
    },
)
