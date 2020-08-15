/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-036_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* agent_test.cpp / 2-036_planck                                              */
/*----------------------------------------------------------------------------*/
//
// Agent and multi-agent problem related structures - testing program.
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

#include "util/statistics.h"

#include "test/agent_test.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void)
    {
	printf("----------------------------------------------------------------\n");
	printf("%s : Agent Test Program\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");
    }

    
    void test_agent_1(const sString &filename)
    {
	printf("Agent test 1 ...\n");
	sUndirectedGraph environment;

	sResult result = environment.from_File_cpf(filename);
	if (sFAILED(result))
	{
	    printf("Reading graph from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading graph from file %s succeeded.\n", filename.c_str());
	}
	environment.to_Screen_vertices();

	sConfiguration init_configuration;
	result = init_configuration.from_File_cpf(filename);
	
	if (sFAILED(result))
	{
	    printf("Reading configuration from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading configuration from file %s succeeded.\n", filename.c_str());
	}
	init_configuration.to_Screen();

	sConfiguration goal_configuration;
	result = goal_configuration.from_File_cpf(filename, 2);
	
	if (sFAILED(result))
	{
	    printf("Reading configuration from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading configuration from file %s succeeded.\n", filename.c_str());
	}
	goal_configuration.to_Screen();

	sInstance instance_A(environment, init_configuration, goal_configuration);	
	instance_A.to_Screen();

	sInstance instance_B;
	result = instance_B.from_File_cpf(filename);
	if (sFAILED(result))
	{
	    printf("Reading instance from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading instance from file %s succeeded.\n", filename.c_str());
	}
	instance_B.to_Screen();

	sInt_32 estimated_total_cost, max_individual_cost;
	estimated_total_cost = instance_B.estimate_TotalPathCost(max_individual_cost);

	printf("Estimated total cost: %d\n", estimated_total_cost);
	printf("Max individual cost: %d\n", max_individual_cost);

	sInstance::MDD_vector MDD, extra_MDD;
	
	sInt_32 extra_cost;
	instance_B.construct_PathMDD(estimated_total_cost, MDD, extra_cost, extra_MDD);

	printf("Agent test 1 ... finished\n");
    }


    void test_agent_2(void)
    {
	printf("Agent test 2 ...\n");
	
	sCommitment commitment_1(4);
	commitment_1.add_Task(1, 4);
	commitment_1.add_Task(1, 5);

	commitment_1.add_Task(2, 2);
	commitment_1.add_Task(2, 1);

	commitment_1.add_Task(4, 1);
	commitment_1.add_Task(4, 6);
	commitment_1.add_Task(4, 7);

	commitment_1.to_Screen();

	printf("Agent test 2 ... finished\n");
    }


    void test_agent_3(const sString &filename)
    {
	printf("Agent test 3 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 0);

	graph_1.to_Screen();
	
	sConfiguration start_configuration(4, 3);
	start_configuration.place_Agent(1, 3);
	start_configuration.place_Agent(2, 1);
	start_configuration.place_Agent(3, 2);	

	start_configuration.to_Screen();

	sCommitment goal_commitment(3);
	goal_commitment.add_Task(1, 0);
	goal_commitment.add_Task(1, 1);

	goal_commitment.add_Task(2, 2);
	goal_commitment.add_Task(2, 3);
	goal_commitment.add_Task(2, 1);

	goal_commitment.add_Task(3, 3);
	goal_commitment.add_Task(3, 2);
	goal_commitment.add_Task(3, 1);		

	goal_commitment.to_Screen();

	sMission mission_1(graph_1, start_configuration, goal_commitment);	
	mission_1.to_Screen();

	mission_1.to_Screen_mHpf();
	mission_1.to_File_mHpf(filename);

	sMission mission_2;
	mission_2.to_Screen();
	mission_2.from_File_mHpf(filename);

	mission_2.to_Screen();
	mission_2.to_Screen_mHpf();
	
	printf("Agent test 3 ... finished\n");	
    }


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
//    test_agent_1("grid_04x04_r04.cpf");
    test_agent_2();
    test_agent_3("hamiltonian_001.mHpf");        
}
