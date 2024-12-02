/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-213_planck                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                  */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* cbs_test.cpp / 2-213_planck                                                */
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

#include "core/graph.h"
#include "core/agent.h"
#include "core/cbs.h"

#include "util/statistics.h"

#include "test/cbs_test.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void)
    {
	printf("----------------------------------------------------------------\n");    
	printf("%s : CBS Test Program\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");
    }

    
    void test_cbs_1(void)
    {
	printf("CBS test 1 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(2, 4);
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 1);
	sConfiguration goal_config(graph_1.get_VertexCount(), 1);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	sCBS::Conflicts_vector Conflicts;
	sCBS::VertexIDs_vector Path;
	sInt_32 path_length = CBS.find_NonconflictingSequence(graph_1, 0, 4, Conflicts, Path);
	
	printf("Path length: %d\n", path_length);
	for (sInt_32 i = 0; i < path_length; ++i)
	{
	    printf("%d ", Path[i]);
	}
	printf("\n");

	printf("CBS test 1 ... finished\n");
    }


    void test_cbs_2(void)
    {
	printf("CBS test 2 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(2, 4);
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 1);
	sConfiguration goal_config(graph_1.get_VertexCount(), 1);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	sCBS::Conflicts_vector Conflicts;
	Conflicts.resize(4);
	Conflicts[1].insert(1);
	Conflicts[1].insert(3);

	Conflicts[3].insert(1);
	Conflicts[3].insert(2);
	Conflicts[3].insert(3);
	
	sCBS::VertexIDs_vector Path;
	sInt_32 path_length = CBS.find_NonconflictingSequence(graph_1, 0, 4, Conflicts, Path);
	
	printf("Path length: %d\n", path_length);
	for (sInt_32 i = 0; i < path_length; ++i)
	{
	    printf("%d ", Path[i]);
	}
	printf("\n");

	printf("CBS test 2 ... finished\n");
    }


    void test_cbs_3(void)
    {
	printf("CBS test 3 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(2, 4);
	graph_1.add_Edge(4, 5);
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 2);
	sConfiguration goal_config(graph_1.get_VertexCount(), 2);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 5);

	goal_config.place_Agent(1, 5);
	goal_config.place_Agent(2, 0);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_NonconflictingPaths(instance_1, agent_Paths, 12);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)	    
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 3 ... finished\n");
    }


    void test_cbs_4(void)
    {
	printf("CBS test 4 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(2, 4);	
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 2);
	sConfiguration goal_config(graph_1.get_VertexCount(), 2);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 4);

	goal_config.place_Agent(1, 4);
	goal_config.place_Agent(2, 0);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_ShortestNonconflictingPaths(instance_1, agent_Paths, 16);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 4 ... finished\n");
    }


    void test_cbs_5(void)
    {
	printf("CBS test 5 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(2, 4);
	graph_1.add_Edge(4, 5);
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 2);
	sConfiguration goal_config(graph_1.get_VertexCount(), 2);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 5);

	goal_config.place_Agent(1, 5);
	goal_config.place_Agent(2, 0);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_ShortestNonconflictingPaths(instance_1, agent_Paths, 1024);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 5 ... finished\n");
    }


    void test_cbs_6(void)
    {
	printf("CBS test 6 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(2, 4);
	graph_1.add_Edge(4, 5);
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 2);
	sConfiguration goal_config(graph_1.get_VertexCount(), 2);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 5);

	goal_config.place_Agent(1, 5);
	goal_config.place_Agent(2, 0);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_ShortestNonconflictingSwapping(instance_1, agent_Paths, 1024);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 6 ... finished\n");
    }


    void test_cbs_7(void)
    {
	printf("CBS test 7 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 3);
	sConfiguration goal_config(graph_1.get_VertexCount(), 3);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	
	goal_config.place_Agent(1, 1);
	goal_config.place_Agent(2, 2);
	goal_config.place_Agent(3, 0);		

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_ShortestNonconflictingSwapping(instance_1, agent_Paths, 12);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 7 ... finished\n");
    }


    void test_cbs_8(void)
    {
	printf("CBS test 8 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);	
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 4);
	sConfiguration goal_config(graph_1.get_VertexCount(), 4);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	start_config.place_Agent(4, 3);	
	
	goal_config.place_Agent(1, 3);
	goal_config.place_Agent(2, 2);
	goal_config.place_Agent(3, 1);
	goal_config.place_Agent(4, 0);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_ShortestNonconflictingSwapping(instance_1, agent_Paths, 32);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 8 ... finished\n");
    }


    void test_cbs_9(void)
    {
	printf("CBS test 9 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(3, 0);
	graph_1.add_Edge(3, 1);
	graph_1.add_Edge(3, 2);
	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(3, 5);
	graph_1.add_Edge(3, 6);		
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 3);
	sConfiguration goal_config(graph_1.get_VertexCount(), 3);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	
	goal_config.place_Agent(1, 6);
	goal_config.place_Agent(2, 5);
	goal_config.place_Agent(3, 4);

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	instance_1.to_Screen();

	sCBS::AgentPaths_vector agent_Paths;
	sInt_32 paths_cost = CBS.find_ShortestNonconflictingPaths_DeltaStar(instance_1, agent_Paths, 128);
	
	printf("Paths cost: %d\n", paths_cost);
	if (paths_cost >= 0)
	{
	    for (sInt_32 agent_id = 1; agent_id < agent_Paths.size(); ++agent_id)
	    {
		sInt_32 path_length = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	printf("CBS test 9 ... finished\n");
    }


    void test_hamiltonian_1(void)
    {
	printf("CBS test - Hamiltonian 1 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 0);
	graph_1.add_Edge(1, 4);
	graph_1.add_Edge(2, 4);		
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 4);
	sConfiguration goal_config(graph_1.get_VertexCount(), 4);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	start_config.place_Agent(4, 3);	
	
	goal_config.place_Agent(1, 3);
	goal_config.place_Agent(2, 2);
	goal_config.place_Agent(3, 1);
	goal_config.place_Agent(4, 0);	

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	sCBS::VertexIDs_vector sink_IDs;
	sink_IDs.push_back(1);
	sink_IDs.push_back(4);
	sink_IDs.push_back(2);

	sCBS::Conflicts_vector vertex_Conflicts;
	sCBS::EdgeConflicts_vector edge_Conflicts;

	sCBS::AgentPaths_vector agent_Paths;
	sCBS::VertexIDs_vector Path;

	sInt_32 hamilton_cost = CBS.findSuperStar_NonconflictingHamiltonian(graph_1,
									    0,
									    sink_IDs,
									    1024,
									    32,
									    vertex_Conflicts,
									    edge_Conflicts,
									    Path);
	
	printf("Hamilton cost: %d\n", hamilton_cost);
	if (hamilton_cost >= 0)
	{
	    sInt_32 path_length = Path.size();
	    for (sInt_32 i = 0; i < path_length; ++i)
	    {
		printf("%d ", Path[i]);
	    }
	    printf("\n");
	}

	printf("CBS test - Hamiltonian 1 ... finished\n");	
    }        


    void test_hamiltonian_2(void)
    {
	printf("CBS test - Hamiltonian 2 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();	

	graph_1.to_Screen();
	graph_1.add_Edge(0, 3);
	graph_1.add_Edge(1, 4);
	graph_1.add_Edge(2, 5);
	
	graph_1.add_Edge(3, 6);
	graph_1.add_Edge(4, 7);
	graph_1.add_Edge(5, 8);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(6, 7);

	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(4, 5);
	graph_1.add_Edge(7, 8);			
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 4);
	sConfiguration goal_config(graph_1.get_VertexCount(), 4);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	start_config.place_Agent(4, 3);	
	
	goal_config.place_Agent(1, 3);
	goal_config.place_Agent(2, 2);
	goal_config.place_Agent(3, 1);
	goal_config.place_Agent(4, 0);	

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	sCBS::VertexIDs_vector sink_IDs;
	sink_IDs.push_back(4);
	sink_IDs.push_back(5);
	sink_IDs.push_back(1);	
//	sink_IDs.push_back(8);
//	sink_IDs.push_back(6);	

	sCBS::Conflicts_vector vertex_Conflicts;
	sCBS::EdgeConflicts_vector edge_Conflicts;

	sCBS::AgentPaths_vector agent_Paths;
	sCBS::VertexIDs_vector Path;

	sInt_32 hamilton_cost = CBS.findSuperStar_NonconflictingHamiltonian(graph_1,
									    0,
									    sink_IDs,
									    1024,
									    32,
									    vertex_Conflicts,
									    edge_Conflicts,
									    Path);
	
	printf("Hamilton cost: %d\n", hamilton_cost);
	
	if (hamilton_cost >= 0)
	{
	    sInt_32 path_length = Path.size();
	    for (sInt_32 i = 0; i < path_length; ++i)
	    {
		printf("%d ", Path[i]);
	    }
	    printf("\n");
	}

	printf("CBS test - Hamiltonian 2 ... finished\n");	
    }


    void test_hamiltonian_3(void)
    {
	printf("CBS test - Hamiltonian 3 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	
	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	
	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.add_Vertex();	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();		

	graph_1.to_Screen();
	graph_1.add_Edge(0, 4);
	graph_1.add_Edge(1, 5);
	graph_1.add_Edge(2, 6);
	graph_1.add_Edge(3, 7);

	graph_1.add_Edge(4, 8);
	graph_1.add_Edge(5, 9);
	graph_1.add_Edge(6, 10);
	graph_1.add_Edge(7, 11);

	graph_1.add_Edge(8, 12);
	graph_1.add_Edge(9, 13);
	graph_1.add_Edge(10, 14);
	graph_1.add_Edge(11, 15);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(4, 5);
	graph_1.add_Edge(8, 9);
	graph_1.add_Edge(12, 13);

	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(5, 6);
	graph_1.add_Edge(9, 10);
	graph_1.add_Edge(13, 14);

	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(6, 7);
	graph_1.add_Edge(10, 11);
	graph_1.add_Edge(14, 15);	
	
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 4);
	sConfiguration goal_config(graph_1.get_VertexCount(), 4);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	start_config.place_Agent(4, 3);	
	
	goal_config.place_Agent(1, 3);
	goal_config.place_Agent(2, 2);
	goal_config.place_Agent(3, 1);
	goal_config.place_Agent(4, 0);	

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	sCBS::VertexIDs_vector sink_IDs;
/*
	sink_IDs.push_back(9);
	sink_IDs.push_back(3);
	sink_IDs.push_back(5);
*/
	sink_IDs.push_back(6);
	sink_IDs.push_back(1);
	sink_IDs.push_back(5);
	sink_IDs.push_back(4);
	sink_IDs.push_back(10);
	sink_IDs.push_back(11);		
	
//	sink_IDs.push_back(8);

	sCBS::Conflicts_vector vertex_Conflicts;
	sCBS::EdgeConflicts_vector edge_Conflicts;

	sCBS::AgentPaths_vector agent_Paths;
	sCBS::VertexIDs_vector Path;

	sInt_32 cost = 6;	

	sInt_32 hamilton_cost = CBS.findSuperStar_NonconflictingHamiltonian(graph_1,
									    0,
									    sink_IDs,
									    cost,
									    0,
									    vertex_Conflicts,
									    edge_Conflicts,
									    Path);
	
	printf("Hamilton cost: %d\n", hamilton_cost);
	
	if (hamilton_cost >= 0)
	{
	    sInt_32 path_length = Path.size();
	    for (sInt_32 i = 0; i < path_length; ++i)
	    {
		printf("%d ", Path[i]);
	    }
	    printf("\n");
	}
	else
	{
	    printf("Oh, no! Cost %d is not enough, everything is wasted!\n", cost);
	}
	printf("Press ENTER, I will try different cost ...\n");
	getchar();

	cost = 7;

	hamilton_cost = CBS.findSuperStar_NonconflictingHamiltonian(graph_1,
								    0,
								    sink_IDs,
								    cost,
								    0,
								    vertex_Conflicts,
								    edge_Conflicts,
								    Path);
	
	printf("Hamilton cost: %d\n", hamilton_cost);
	
	if (hamilton_cost >= 0)
	{
	    sInt_32 path_length = Path.size();
	    for (sInt_32 i = 0; i < path_length; ++i)
	    {
		printf("%d ", Path[i]);
	    }
	    printf("\n");
	}
	else
	{
	    printf("Oh, no! Cost %d is not enough, everything is wasted!\n", cost);
	}

	printf("CBS test - Hamiltonian 3 ... finished\n");	
    }


    void test_hamiltonian_4(void)
    {
	printf("CBS test - Hamiltonian 4 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	
	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();	
	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.add_Vertex();	
	graph_1.add_Vertex();	
	graph_1.add_Vertex();
	graph_1.add_Vertex();		

	graph_1.to_Screen();
	graph_1.add_Edge(0, 4);
	graph_1.add_Edge(1, 5);
	graph_1.add_Edge(2, 6);
	graph_1.add_Edge(3, 7);

	graph_1.add_Edge(4, 8);
	graph_1.add_Edge(5, 9);
	graph_1.add_Edge(6, 10);
	graph_1.add_Edge(7, 11);

	graph_1.add_Edge(8, 12);
	graph_1.add_Edge(9, 13);
	graph_1.add_Edge(10, 14);
	graph_1.add_Edge(11, 15);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(4, 5);
	graph_1.add_Edge(8, 9);
	graph_1.add_Edge(12, 13);

	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(5, 6);
	graph_1.add_Edge(9, 10);
	graph_1.add_Edge(13, 14);

	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(6, 7);
	graph_1.add_Edge(10, 11);
	graph_1.add_Edge(14, 15);	
	
	graph_1.to_Screen();

	sConfiguration start_config(graph_1.get_VertexCount(), 4);
	sConfiguration goal_config(graph_1.get_VertexCount(), 4);

	start_config.place_Agent(1, 0);
	start_config.place_Agent(2, 1);
	start_config.place_Agent(3, 2);
	start_config.place_Agent(4, 3);	
	
	goal_config.place_Agent(1, 3);
	goal_config.place_Agent(2, 2);
	goal_config.place_Agent(3, 1);
	goal_config.place_Agent(4, 0);	

	sInstance instance_1(graph_1, start_config, goal_config);
	sCBS CBS(&instance_1);

	sCBS::VertexIDs_vector source_IDs, sink_IDs, endpoint_IDs;

	source_IDs.push_back(0);
/*
	sink_IDs.push_back(9);
	sink_IDs.push_back(3);
	sink_IDs.push_back(5);
*/
	sink_IDs.push_back(6);
	sink_IDs.push_back(1);
	sink_IDs.push_back(5);
	sink_IDs.push_back(4);
	sink_IDs.push_back(10);
	sink_IDs.push_back(11);		
	
//	sink_IDs.push_back(8);

	sCBS::Conflicts_vector vertex_Conflicts;
	sCBS::EdgeConflicts_vector edge_Conflicts;

	sCBS::AgentPaths_vector agent_Paths;
	sCBS::VertexIDs_vector Path;

	endpoint_IDs = sink_IDs;
	endpoint_IDs.push_back(0);

//	graph_1.calc_SourceGoalShortestPaths(source_IDs, sink_IDs);
	graph_1.calc_SourceGoalShortestPaths(CBS.m_source_Distances, CBS.m_goal_Distances, source_IDs, sink_IDs);
	graph_1.calc_EndpointShortestPaths(endpoint_IDs);	

	sInt_32 cost = 10;

	sInt_32 hamilton_cost = CBS.findUltraStar_NonconflictingHamiltonian(graph_1,
									    0,
									    sink_IDs,
									    cost,
									    0,
									    vertex_Conflicts,
									    edge_Conflicts,
									    Path);
	
	printf("Hamilton cost: %d\n", hamilton_cost);
	
	if (hamilton_cost >= 0)
	{
	    sInt_32 path_length = Path.size();
	    for (sInt_32 i = 0; i < path_length; ++i)
	    {
		printf("%d ", Path[i]);
	    }
	    printf("\n");
	}
	else
	{
	    printf("Oh, no! Cost %d is not enough, everything is wasted!\n", cost);
	}
	printf("Press ENTER, I will try different cost ...\n");
	getchar();

	cost = 21;

	hamilton_cost = CBS.findUltraStar_NonconflictingHamiltonian(graph_1,
								    0,
								    sink_IDs,
								    cost,
								    0,
								    vertex_Conflicts,
								    edge_Conflicts,
								    Path);
	
	printf("Hamilton cost: %d\n", hamilton_cost);
	
	if (hamilton_cost >= 0)
	{
	    sInt_32 path_length = Path.size();
	    for (sInt_32 i = 0; i < path_length; ++i)
	    {
		printf("%d ", Path[i]);
	    }
	    printf("\n");
	}
	else
	{
	    printf("Oh, no! Cost %d is not enough, everything is wasted!\n", cost);
	}

	printf("CBS test - Hamiltonian 4 ... finished\n");	
    }                    
    

/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    
//    test_cbs_1();
//    test_cbs_2();
//    test_cbs_3();
//    test_cbs_4();
//    test_cbs_5();
//    test_cbs_6();
//    test_cbs_7();
//    test_cbs_8();
//    test_cbs_9();
    /*
    test_hamiltonian_1();
    test_hamiltonian_2();
    test_hamiltonian_3();
    */
    test_hamiltonian_4();            
}
