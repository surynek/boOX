/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-132_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* graph_test.cpp / 2-132_planck                                              */
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
#include "core/cbs.h"
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


    void test_undirected_graph_5(void)
    {
	printf("Undirecteg graph test 5 ...\n");
	sUndirectedGraph grid_graph_A(4, 4);
	grid_graph_A.to_Screen();

	sUndirectedGraph grid_graph_B(4, 4);
	grid_graph_B.to_Screen();	

	sUndirectedGraph::VertexIDs_vector endpoint_A_IDs;
	endpoint_A_IDs.push_back(0);
	endpoint_A_IDs.push_back(7);
	endpoint_A_IDs.push_back(13);
	endpoint_A_IDs.push_back(8);

	sUndirectedGraph::VertexIDs_vector endpoint_B_IDs;	
	endpoint_B_IDs.push_back(14);
	endpoint_B_IDs.push_back(11);
	endpoint_B_IDs.push_back(4);
	endpoint_B_IDs.push_back(12);

	grid_graph_A.calc_EndpointShortestPaths(endpoint_A_IDs);
	sInt_32 span_cost_A = grid_graph_A.calc_MinimumSpanningTree(endpoint_A_IDs);
	
	grid_graph_B.calc_EndpointShortestPaths(endpoint_B_IDs);
	sInt_32 span_cost_B = grid_graph_B.calc_MinimumSpanningTree(endpoint_B_IDs);

	printf("Spanning tree A cost: %d\n", span_cost_A);
	printf("Spanning tree B cost: %d\n", span_cost_B);

	printf("Undirecteg graph test 5 ... finished\n");
    }

    
    void test_undirected_graph_6(void)
    {
	printf("Undirecteg graph test 6 ...\n");
	sUndirectedGraph grid_graph_A(4, 4);
	grid_graph_A.to_Screen();

	sUndirectedGraph grid_graph_B(4, 4);
	grid_graph_B.to_Screen();	

	sUndirectedGraph::VertexIDs_vector endpoint_A_IDs;
	endpoint_A_IDs.push_back(0);
	endpoint_A_IDs.push_back(7);
	endpoint_A_IDs.push_back(13);
	endpoint_A_IDs.push_back(8);

	sUndirectedGraph::VertexIDs_vector endpoint_B_IDs;	
	endpoint_B_IDs.push_back(14);
	endpoint_B_IDs.push_back(11);
	endpoint_B_IDs.push_back(4);
	endpoint_B_IDs.push_back(12);

	sUndirectedGraph::VertexIDs_vector source_IDs, sink_IDs, endpoint_IDs;	
	sCBS CBS_1((sInstance*)NULL);
	sink_IDs = endpoint_A_IDs;
	source_IDs.push_back(1);	
	grid_graph_A.calc_SourceGoalShortestPaths(CBS_1.m_source_Distances, CBS_1.m_goal_Distances, source_IDs, sink_IDs);	
	grid_graph_A.calc_EndpointShortestPaths(endpoint_A_IDs);
	
	sInt_32 hamilton_cost_A = grid_graph_A.calc_MinimumHamiltonianPath(CBS_1, 1, endpoint_A_IDs);

	sCBS CBS_2((sInstance*)NULL);
	sink_IDs.clear();
	source_IDs.clear();
	sink_IDs = endpoint_B_IDs;
	source_IDs.push_back(2);	
	grid_graph_B.calc_SourceGoalShortestPaths(CBS_1.m_source_Distances, CBS_2.m_goal_Distances, source_IDs, sink_IDs);	
	
	grid_graph_B.calc_EndpointShortestPaths(endpoint_B_IDs);
	sInt_32 hamilton_cost_B = grid_graph_B.calc_MinimumHamiltonianPath(CBS_2, 2, endpoint_B_IDs);

	printf("Hamiltonian path A cost: %d\n", hamilton_cost_A);
	printf("Hamiltonian path B cost: %d\n", hamilton_cost_B);

	printf("Undirecteg graph test 6 ... finished\n");
    }


    void test_undirected_graph_7(void)
    {
	printf("Undirecteg graph test 7 ...\n");
	sUndirectedGraph grid_graph(4, 4);
	grid_graph.to_Screen();

	sUndirectedGraph::VertexIDs_vector endpoint_IDs;
	endpoint_IDs.push_back(0);
	endpoint_IDs.push_back(7);
	endpoint_IDs.push_back(15);
	endpoint_IDs.push_back(8);

	grid_graph.calc_AllPairsShortestPaths();
	grid_graph.m_source_Distances = grid_graph.m_all_pairs_Distances;
	grid_graph.m_goal_Distances = grid_graph.m_all_pairs_Distances;
	printf("goal: %ld\n", grid_graph.m_goal_Distances.size());
	grid_graph.m_endpoint_Distances = grid_graph.m_all_pairs_Distances;
	grid_graph.calc_HamiltonianCosts(1, endpoint_IDs);

	for (sUndirectedGraph::Distances_vector::const_iterator hc = grid_graph.m_hamiltonian_Costs.begin(); hc != grid_graph.m_hamiltonian_Costs.end(); ++hc)
	{
	    printf("%d ", *hc);
	}
	printf("\n");

	printf("Undirecteg graph test 7 ... finished\n");
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

	++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;

	s_GlobalStatistics.to_Screen();
	s_GlobalStatistics.enter_Phase("extreme");

	++s_GlobalStatistics.get_CurrentPhase().m_micro_search_Steps;
	++s_GlobalStatistics.get_CurrentPhase().m_micro_search_Steps;

	s_GlobalStatistics.to_Screen();
    }


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    /*
    test_undirected_graph_1();

    test_undirected_graph_2(16, 0.1);
    test_undirected_graph_2(16, 0.5);
    test_undirected_graph_2(32, 0.6);
    test_undirected_graph_2(64, 0.9);

    test_undirected_graph_3("graph_01.txt");
    test_statistics_1();

    test_undirected_graph_4();
    */
    test_undirected_graph_5();
    getchar();
    test_undirected_graph_6();
    getchar();   
    test_undirected_graph_7();
}


