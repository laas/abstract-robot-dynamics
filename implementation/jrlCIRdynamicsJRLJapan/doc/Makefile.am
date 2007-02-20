# 
#  Copyright 
# 


jrlCIRdynamicsJRLJapandir = $(datadir)/doc/jrlCIRdynamicsJRLJapan

EXTRA_DIST = \
	header.html \
	footer.html \
	package.css \
	pictures/footer.jpg \
	additionalDoc/package.h 


all:html

install: 
	mkdir -p $(jrlCIRdynamicsJRLJapandir)
	$(INSTALL_DATA) html/* $(jrlCIRdynamicsJRLJapandir)
	mkdir -p $(jrlCIRdynamicsJRLJapandir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(jrlCIRdynamicsJRLJapandir)/pictures

html: package.dox
	doxygen $<
