all: debug

debug:	libGlucose_boOX_dbg.a 

optimized:	libGlucose_boOX_opt.a 

libGlucose_boOX_dbg.a:	Options.o_dbg Solver.o_dbg System.o_dbg 
	ar r libGlucose_boOX_dbg.a Options.o_dbg Solver.o_dbg System.o_dbg

Options.o_dbg:	Alg.h Alloc.h BoundedQueue.h Constants.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Options.cc
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/glucose -I../../src/util -oOptions.o_dbg Options.cc

Solver.o_dbg:	Alg.h Alloc.h BoundedQueue.h Constants.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Solver.cc
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/glucose -I../../src/util -oSolver.o_dbg Solver.cc

System.o_dbg:	Alg.h Alloc.h BoundedQueue.h Constants.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h System.cc
	g++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c -I../../src -I../../src/glucose -I../../src/util -oSystem.o_dbg System.cc


libGlucose_boOX_opt.a:	Options.o_opt Solver.o_opt System.o_opt 
	ar r libGlucose_boOX_opt.a Options.o_opt Solver.o_opt System.o_opt

Options.o_opt:	Alg.h Alloc.h BoundedQueue.h Constants.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Options.cc
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/glucose -I../../src/util -oOptions.o_opt Options.cc

Solver.o_opt:	Alg.h Alloc.h BoundedQueue.h Constants.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Solver.cc
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/glucose -I../../src/util -oSolver.o_opt Solver.cc

System.o_opt:	Alg.h Alloc.h BoundedQueue.h Constants.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h System.cc
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/glucose -I../../src/util -oSystem.o_opt System.cc


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output*.txt
