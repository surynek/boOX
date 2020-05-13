all: debug

debug:	libCore_boOX_dbg.a 

optimized:	libCore_boOX_opt.a 

libCore_boOX_dbg.a:	graph.o_dbg agent.o_dbg smtcbs.o_dbg cbs.o_dbg icts.o_dbg cnf.o_dbg mapR.o_dbg kruhoR.o_dbg cbsR.o_dbg smtcbsR.o_dbg 
	ar r libCore_boOX_dbg.a graph.o_dbg agent.o_dbg smtcbs.o_dbg cbs.o_dbg icts.o_dbg cnf.o_dbg mapR.o_dbg kruhoR.o_dbg cbsR.o_dbg smtcbsR.o_dbg

graph.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h graph.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -ograph.o_dbg graph.cpp

agent.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h agent.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -oagent.o_dbg agent.cpp

smtcbs.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h smtcbs.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -osmtcbs.o_dbg smtcbs.cpp

cbs.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h cbs.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -ocbs.o_dbg cbs.cpp

icts.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h icts.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -oicts.o_dbg icts.cpp

cnf.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h cnf.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -ocnf.o_dbg cnf.cpp

mapR.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h mapR.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -omapR.o_dbg mapR.cpp

kruhoR.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h kruhoR.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -okruhoR.o_dbg kruhoR.cpp

cbsR.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h cbsR.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -ocbsR.o_dbg cbsR.cpp

smtcbsR.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h smtcbsR.cpp
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/core -osmtcbsR.o_dbg smtcbsR.cpp


libCore_boOX_opt.a:	graph.o_opt agent.o_opt smtcbs.o_opt cbs.o_opt icts.o_opt cnf.o_opt mapR.o_opt kruhoR.o_opt cbsR.o_opt smtcbsR.o_opt 
	ar r libCore_boOX_opt.a graph.o_opt agent.o_opt smtcbs.o_opt cbs.o_opt icts.o_opt cnf.o_opt mapR.o_opt kruhoR.o_opt cbsR.o_opt smtcbsR.o_opt

graph.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h graph.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ograph.o_opt graph.cpp

agent.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h agent.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oagent.o_opt agent.cpp

smtcbs.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h smtcbs.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -osmtcbs.o_opt smtcbs.cpp

cbs.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h cbs.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ocbs.o_opt cbs.cpp

icts.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h icts.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oicts.o_opt icts.cpp

cnf.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h cnf.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ocnf.o_opt cnf.cpp

mapR.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h mapR.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -omapR.o_opt mapR.cpp

kruhoR.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h kruhoR.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -okruhoR.o_opt kruhoR.cpp

cbsR.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h cbsR.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ocbsR.o_opt cbsR.cpp

smtcbsR.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h graph.h agent.h smtcbs.h cbs.h icts.h cnf.h mapR.h kruhoR.h cbsR.h smtcbsR.h smtcbsR.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -osmtcbsR.o_opt smtcbsR.cpp


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output*.txt
