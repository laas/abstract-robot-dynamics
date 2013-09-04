abstract-robot-dynamics
=======================

[![Build Status](https://travis-ci.org/laas/abstract-robot-dynamics.png)](https://travis-ci.org/laas/abstract-robot-dynamics)
[![Coverage Status](https://coveralls.io/repos/laas/abstract-robot-dynamics/badge.png?branch=master)](https://coveralls.io/r/laas/abstract-robot-dynamics?branch=master)

This software provides interfaces which allow a generic representation
of the dynamic of a humanoid robot.

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The package depends on several packages which have to be available on
your machine.

 - Libraries:
   - [jrl-mal][jrl-mal] (>=1.7.4)
     This package is shipped with a pkg-config file which is used for their
     detection. Make sure your `PKG_CONFIG_PATH` is correct if CMake cannot
     detect it.
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)

[jrl-mal]: http://github.com/jrl-umi3218/jrl-mal "jrl-mal"
