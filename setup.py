from setuptools import setup, find_packages

long_description = """
A fast and scalable motion planning framework for tasks expressed in Linear Temporal Logic (LTL).

Accompanies the paper "Temporal Logic Motion Planning with Convex Optimization
via Graphs of Convex Sets" by Vince Kurtz and Hai Lin.
"""

setup(name="ltlgcs",
        version="0.0.1",
        description="Fast and scalable Linear Temporal Logic (LTL) motion planning",
        long_description=long_description,
        url="https://github.com/vincekurtz/ltl_gcs",
        author="Vince Kurtz",
        author_email="vjkurtz@gmail.com",
        license="MIT",
        packages=find_packages(),
        python_requires=">=3.8",
        install_requires=[
            "pydrake",
            "ltlf2dfa",
            "treelib",
            "matplotlib",
            "scipy",
            "sympy",
            "numpy",
            "graphviz",
            "pydot"],
        zip_safe=False)

