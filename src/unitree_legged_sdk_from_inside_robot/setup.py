import os
from setuptools import setup

# amarco: made a Python package to expose the data logging files and other things

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "unitree_legged_sdk_python_tools",
    version = "0.0",
    author = "",
    author_email = "",
    description = (""),
    keywords = "",
    packages=[	'unitree_legged_sdk_python_tools',
                'unitree_legged_sdk_python_tools.utils'],
    long_description=read('README.md'),
)