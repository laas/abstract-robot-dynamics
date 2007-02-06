# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

robotDynamicsDocdir = $(datadir)/doc/abstractRobotDynamics

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/robotDynamics.h \
	pictures/HRP2.jpg

all: html

install:
	mkdir -p $(robotDynamicsDocdir)
	$(INSTALL_DATA) html/* $(robotDynamicsDocdir)
	mkdir -p $(robotDynamicsDocdir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(robotDynamicsDocdir)/pictures

html: robotDynamics.dox
	doxygen $<
