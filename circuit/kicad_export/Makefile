# brush up kicad export for hackvana import

# TODO: add a check to point to out-of-date files
# e.g. if only drill file was exported and everything else is out-of-date



# TODO: find the basename automatically
# base filename (project name), no blanks allowed!
BASENAME=tasta

# try to find out current hardware revision (so add a tag _before_ calling this makefile!)
REVISION=$(shell git describe --tags)

ZIPFILE=$(BASENAME)_$(REVISION).zip

all:	rename zip

rename:
	mv $(BASENAME).drl           $(BASENAME).plated-drill.txt
	mv $(BASENAME)-F_Cu.gtl      $(BASENAME).frontmask.gts
	mv $(BASENAME)-F_SilkS.gto   $(BASENAME).frontsilk.gbo
	mv $(BASENAME)-Edge_Cuts.gbr $(BASENAME).outline.gm1
	mv $(BASENAME)-B_Cu.gbl      $(BASENAME).backmask.gbs
	mv $(BASENAME)-B_SilkS.gbo   $(BASENAME).backsilk.gbo

zip:
	[ "$(REVISION)" ] || ( echo "no current revision found!" ; false )
	zip $(ZIPFILE) $(BASENAME).*.*
