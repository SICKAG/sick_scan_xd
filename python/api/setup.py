"""Setup file"""

import setuptools

repo_url = "https://github.com/SICKAG/sick_scan_xd"

if __name__ == "__main__":
    # with open("README.md", "r", encoding="utf-8") as fh:
    #     long_description = fh.read()

    setuptools.setup(
        name="sick_scan_api",
        python_requires=">=3.8",
        install_requires=[
            "numpy"
        ],
        entry_points={
            "console_scripts": [
            ],
        },
    )
