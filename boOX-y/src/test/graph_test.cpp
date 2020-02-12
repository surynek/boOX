/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 1-224_leibniz                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* graph_test.cpp / 1-224_leibniz                                             */
/*----------------------------------------------------------------------------*/
//
// Graph data structures and algorithms - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

#include "common/types.h"
#include "core/graph.h"
#include "core/agent.h"
#include "util/statistics.h"

#include "test/graph_test.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void)
    {
	printf("----------------------------------------------------------------\n");    
	printf("%s : Graph Test Program\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");
    }

    
    void test_undirected_graph_1(void)
    {
	printf("Undirecteg graph test 1 ...\n");

	sUndirectedGraph graph_1;

	graph_1.to_Screen();
	graph_1.add_Vertex();
	graph_1.to_Screen();

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.add_Vertices(10);
	graph_1.to_Screen();

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 2);
	graph_1.to_Screen();

	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(2, 4);
	graph_1.add_Edge(7, 4);
	graph_1.add_Edge(8, 4);
	graph_1.add_Edge(9, 4);
	graph_1.add_Edge(10, 4);
	graph_1.add_Edge(11, 4);
	graph_1.to_Screen();

	printf("Undirecteg graph test 1 ... finished\n");
    }


    void test_undirected_graph_2(int N_Vertices, double edge_prob)
    {
	printf("Undirecteg graph test 2 ...\n");

	sUndirectedGraph graph_2;
	graph_2.to_Screen();
	graph_2.add_Vertices(N_Vertices);
	graph_2.to_Screen();

	for (int i = 0; i < N_Vertices - 1; ++i)
	{
	    for (int j = i + 1; j < N_Vertices; ++j)
	    {
		double rnd = rand() / (double)RAND_MAX;
		if (rnd < edge_prob)
		{
		    graph_2.add_Edge(i, j);
		}
	    }
	}

	graph_2.to_Screen_vertices();

	printf("Undirecteg graph test 2 ... finished\n");
    }


    void test_undirected_graph_3(const sString &filename)
    {
	printf("Undirecteg graph test 3 ...\n");
	sUndirectedGraph graph_3;

	sResult result = graph_3.from_File_cpf(filename);
	if (sFAILED(result))
	{
	    printf("Reading graph from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading graph from file %s succeeded.\n", filename.c_str());
	}
	graph_3.to_Screen_vertices();

	sConfiguration configuration;
	result = configuration.from_File_cpf(filename);
	if (sFAILED(result))
	{
	    printf("Reading configuration from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading configuration from file %s succeeded.\n", filename.c_str());
	}
	configuration.to_Screen();

	sSolution solution;
	result = solution.from_File_cpf(filename);
	if (sFAILED(result))
	{
	    printf("Reading solution from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading solution from file %s succeeded.\n", filename.c_str());
	}
	solution.to_Screen();

	printf("Undirecteg graph test 3 ... finished\n");
    }


    void test_undirected_graph_4(void)
    {
	printf("Undirecteg graph test 4 ...\n");
	sUndirectedGraph graph_1(2, 2);
	graph_1.to_Screen();

	sUndirectedGraph graph_2(2, 3);
	graph_2.to_Screen(); 

	sUndirectedGraph graph_3(3, 3);
	graph_3.to_Screen();

	sUndirectedGraph graph_4(5, 5, 0.2);
	graph_4.to_Screen();

	sConfiguration configuration_1(10, 5);
	configuration_1.to_Screen();

	sConfiguration configuration_2(10, 5, true);
	configuration_2.to_Screen();

	printf("Undirecteg graph test 4 ... finished\n");
    }


    void test_statistics_1(void)
    {
	double wc_start = sStatistics::get_WC_Seconds();
	double cpu_start = sStatistics::get_CPU_Seconds();

	int j = 1;
	for (int i  = 0; i < 100; ++i)
	{
	    system("ls");
	    for (int ii  = 0; ii < 100000; ++ii)
	    {
		j *= i * ii;
	    }
	}

	double wc_finish = sStatistics::get_WC_Seconds();
	double cpu_finish = sStatistics::get_CPU_Seconds();
	printf("Wall clock time: %.3f\n", wc_finish - wc_start);
	printf("CPU time       : %.3f\n", cpu_finish - cpu_start);

	++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;

	s_GlobalStatistics.to_Screen();
	s_GlobalStatistics.enter_Phase("extreme");

	++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;

	s_GlobalStatistics.to_Screen();
    }


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    test_undirected_graph_1();

    test_undirected_graph_2(16, 0.1);
    test_undirected_graph_2(16, 0.5);
    test_undirected_graph_2(32, 0.6);
    test_undirected_graph_2(64, 0.9);

    test_undirected_graph_3("graph_01.txt");
    test_statistics_1();

    test_undirected_graph_4();
}


