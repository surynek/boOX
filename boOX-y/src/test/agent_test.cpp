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
/* agent_test.cpp / 1-224_leibniz                                             */
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


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    test_agent_1("grid_04x04_r04.cpf");
}
