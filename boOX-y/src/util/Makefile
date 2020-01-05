all: debug

debug:	libUtil_boOX_dbg.a 

optimized:	libUtil_boOX_opt.a 

libUtil_boOX_dbg.a:	statistics.o_dbg io.o_dbg 
	ar r libUtil_boOX_dbg.a statistics.o_dbg io.o_dbg

statistics.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h statistics.h io.h statistics.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/util -ostatistics.o_dbg statistics.cpp

io.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h statistics.h io.h io.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/util -oio.o_dbg io.cpp


libUtil_boOX_opt.a:	statistics.o_opt io.o_opt 
	ar r libUtil_boOX_opt.a statistics.o_opt io.o_opt

statistics.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h statistics.h io.h statistics.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/util -ostatistics.o_opt statistics.cpp

io.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h statistics.h io.h io.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/util -oio.o_opt io.cpp


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output*.txt
