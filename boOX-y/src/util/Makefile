all: debug

debug:	libUtil_boOX_dbg.a 

optimized:	libUtil_boOX_opt.a 

libUtil_boOX_dbg.a:	statistics.o_dbg 
	ar r libUtil_boOX_dbg.a statistics.o_dbg

statistics.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h statistics.h statistics.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/util -ostatistics.o_dbg statistics.cpp


libUtil_boOX_opt.a:	statistics.o_opt 
	ar r libUtil_boOX_opt.a statistics.o_opt

statistics.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h statistics.h statistics.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/util -ostatistics.o_opt statistics.cpp


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output*.txt
