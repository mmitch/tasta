# brush up kicad export for hackvana import

# TODO: add a check to point to out-of-date files
# e.g. if only drill file was exported and everything else is out-of-date



# TODO: find the basename automatically
# base filename (project name), no blanks allowed!
BASENAME=tasta

# try to find out current hardware revision (so add a tag _before_ calling this makefile!)
REVISION=$(shell git describe --tags)

# switch the format of the string
PCBREVISION=$(subst _,. ,$(REVISION))

# needed to check if the revision matches the PCB
PCBFILE=../kicad_source/tasta.kicad_pcb

# also check the revision on the schematics and exported schematics
SCHEMAFILE_SCH=../kicad_source/tasta.sch
SCHEMAFILE_PDF=../tasta.pdf

ZIPFILE=$(BASENAME)_$(REVISION).zip

all:	rename zip

rename:
	mv $(BASENAME).drl           $(BASENAME).plated-drill.txt
	mv $(BASENAME)-F_SilkS.gto   $(BASENAME).frontsilk.gto
	mv $(BASENAME)-F_Mask.gts    $(BASENAME).frontmask.gts
	mv $(BASENAME)-F_Cu.gtl      $(BASENAME).front.gtl
	mv $(BASENAME)-Edge_Cuts.gbr $(BASENAME).outline.gm1
	mv $(BASENAME)-B_SilkS.gbo   $(BASENAME).backsilk.gbo
	mv $(BASENAME)-B_Mask.gbs    $(BASENAME).backmask.gbs
	mv $(BASENAME)-B_Cu.gbl      $(BASENAME).back.gbl

check:
	[ "$(REVISION)" ] || ( echo "no current revision found!" ; false )
	grep -q "$(PCBREVISION)" $(PCBFILE) || (echo "revision on PCB does not match!" ; false )
	grep -q "Rev \"$(REVISION)\"" $(SCHEMAFILE_SCH) || (echo "revision on SCH does not match!" ; false )
	pdftotext $(SCHEMAFILE_PDF) - | grep -q "Rev: $(REVISION)" || (echo "revision on PDF does not match!" ; false )

zip:	check
	zip $(ZIPFILE) $(BASENAME).*.*
