See INSTALL for installation instructions.
#
#  Copyright 2009 JRL AIST-CNRS
#
#  Author: Florent Lamiraux
#

Installation instructions for this package
------------------------------------------------

It is recommended to create a specific directory to install this package.
mkdir build
cd build
cmake [Options] ..
make 
make install

Options:

-DCMAKE_INSTALL_PREFIX=...
  specifies the directory where to install the package.
