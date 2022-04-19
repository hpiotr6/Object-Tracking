#!/usr/bin/env python

from setuptools import setup

setup(name='tracker',
      version='1.0',
      # list folders, not files
      packages=['tracker',
                'tracker.test'],
      install_requires=['filterpy',
                        'scipy',
                        'numpy']
      )
