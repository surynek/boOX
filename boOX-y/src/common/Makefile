all: debug

debug:	libCommon_boOX_dbg.a 

optimized:	libCommon_boOX_opt.a 

libCommon_boOX_dbg.a:	types.o_dbg 
	ar r libCommon_boOX_dbg.a types.o_dbg

types.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h types.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/common -otypes.o_dbg types.cpp


libCommon_boOX_opt.a:	types.o_opt 
	ar r libCommon_boOX_opt.a types.o_opt

types.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h types.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/common -otypes.o_opt types.cpp


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output*.txt
