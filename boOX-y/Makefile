SUBDIRS = src/common src/core src/util src/glucose src/test src/main src

all: debug

debug:
	for dir in $(SUBDIRS); do make -C $$dir debug; done

optimized:
	for dir in $(SUBDIRS); do make -C $$dir optimized; done

release:	optimized
download:	optimized

clean:
	for dir in $(SUBDIRS); do make -C $$dir clean; done
	rm -f *~ *.o_dbg *.o_opt *.a *.bak output*.txt
