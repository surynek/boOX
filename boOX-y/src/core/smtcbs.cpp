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
/* smtcbs.cpp / 1-224_leibniz                                                 */
/*----------------------------------------------------------------------------*/
//
// Conflict based search implemented using SAT-modulo theories
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <map>

#include "config.h"
#include "compile.h"
#include "version.h"
#include "defs.h"
#include "result.h"

#include "common/types.h"
#include "core/smtcbs.h"
#include "util/statistics.h"


using namespace std;
using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sSMTCBSBase

    sSMTCBSBase::sSMTCBSBase(sBoolEncoder *solver_Encoder)
	: m_solver_Encoder(solver_Encoder)
    {
	// nothing
    }
    

/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBSBase::alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_1vector &vector_Variables, sInt_32 dim_1)
    {
	sInt_32 variable_ID = first_variable_ID;

	vector_Variables.resize(dim_1);

	for (sInt_32 i = 0; i < dim_1; ++i)
	{
	    vector_Variables[i] = variable_ID++;
	}
	return variable_ID;
    }

    
    sInt_32 sSMTCBSBase::alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_2vector &vector_Variables, sInt_32 dim_1, sInt_32 dim_2)
    {
	sInt_32 variable_ID = first_variable_ID;

	vector_Variables.resize(dim_1);

	for (sInt_32 i = 0; i < dim_1; ++i)
	{
	    variable_ID = alloc_VariableVector(variable_ID, vector_Variables[i], dim_2);
	}
	return variable_ID;
    }


    sInt_32 sSMTCBSBase::alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_3vector &vector_Variables, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3)
    {
	sInt_32 variable_ID = first_variable_ID;

	vector_Variables.resize(dim_1);

	for (sInt_32 i = 0; i < dim_1; ++i)
	{
	    variable_ID = alloc_VariableVector(variable_ID, vector_Variables[i], dim_2, dim_3);
	}
	return variable_ID;
    }


    sInt_32 sSMTCBSBase::alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_4vector &vector_Variables, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3, sInt_32 dim_4)
    {
	sInt_32 variable_ID = first_variable_ID;

	vector_Variables.resize(dim_1);

	for (sInt_32 i = 0; i < dim_1; ++i)
	{
	    variable_ID = alloc_VariableVector(variable_ID, vector_Variables[i], dim_2, dim_3, dim_4);
	}
	return variable_ID;
    }


    sInt_32 sSMTCBSBase::alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_5vector &vector_Variables, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3, sInt_32 dim_4, sInt_32 dim_5)
    {
	sInt_32 variable_ID = first_variable_ID;

	vector_Variables.resize(dim_1);

	for (sInt_32 i = 0; i < dim_1; ++i)
	{
	    variable_ID = alloc_VariableVector(variable_ID, vector_Variables[i], dim_2, dim_3, dim_4, dim_5);
	}
	return variable_ID;
    }    
    


    
/*----------------------------------------------------------------------------*/
// sSMTCBS

    sSMTCBS::sSMTCBS(sBoolEncoder *solver_Encoder, sInstance *instance)
	: sCBSBase(instance)
	, sSMTCBSBase(solver_Encoder)
	, m_subopt_weight(-1)	  
	, m_cbs(instance)

    {
	// nothing
    }

    
    sSMTCBS::sSMTCBS(sBoolEncoder *solver_Encoder, sInstance *instance, sDouble timeout)
	: sCBSBase(instance, timeout)
	, sSMTCBSBase(solver_Encoder)
	, m_subopt_weight(-1)
	, m_cbs(instance)

    {
	// nothing
    }


    sSMTCBS::sSMTCBS(sBoolEncoder *solver_Encoder, sDouble subopt_weight, sInstance *instance)
	: sCBSBase(instance)
	, sSMTCBSBase(solver_Encoder)
	, m_subopt_weight(subopt_weight)
	, m_cbs(instance)

    {
	// nothing
    }

    
    sSMTCBS::sSMTCBS(sBoolEncoder *solver_Encoder, sDouble subopt_weight, sInstance *instance, sDouble timeout)
	: sCBSBase(instance, timeout)
	, sSMTCBSBase(solver_Encoder)
	, m_subopt_weight(subopt_weight)
	, m_cbs(instance)

    {
	// nothing
    }    


/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::find_ShortestNonconflictingSwapping(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwapping(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingSwapping(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingSwapping(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwapping(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwapping(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwapping(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingSwapping(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverse(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwappingInverse(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingSwappingInverse(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwappingInverse(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverse(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    
	    if ((solution_cost = find_NonconflictingSwappingInverse(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverseDepleted(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwappingInverseDepleted(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingSwappingInverseDepleted(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwappingInverseDepleted(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingSwappingInverseDepleted(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    
	    if ((solution_cost = find_NonconflictingSwappingInverseDepleted(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }        

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPaths(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPaths(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPaths(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPaths(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }    

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPaths(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPaths(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPaths(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;
	
        #ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost);
	Context context;
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));		
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPaths(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }
	    
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }	    
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverse(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPathsInverse(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPathsInverse(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }    

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPathsInverse(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverse(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;
	
        #ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost);
	Context context;
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));		
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPathsInverse(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }
	    
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }	    
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseSeparated(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPathsInverseSeparated(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseSeparated(sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPathsInverseSeparated(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }    

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseSeparated(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPathsInverseSeparated(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseSeparated(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	
        #ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost);
	Context context;
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPathsInverseSeparated(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }
	    
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }	    
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseDepleted(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPathsInverseDepleted(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPathsInverseDepleted(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }    

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPathsInverseDepleted(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPathsInverseDepleted(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	
        #ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost);
	Context context;
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));		
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPathsInverseDepleted(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }
	    
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }	    
	}
	return -1;
    }

    
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutation(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutation(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutation(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPermutation(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutation(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPermutation(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverse(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutationInverse(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPermutationInverse(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverse(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPermutationInverse(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverseDepleted(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutationInverseDepleted(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPermutationInverseDepleted(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutationInverseDepleted(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingPermutationInverseDepleted(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPermutationInverseDepleted(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotation(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotation(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingRotation(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingRotation(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotation(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingRotation(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverse(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotationInverse(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingRotationInverse(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotationInverse(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverse(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingRotationInverse(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverseDepleted(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotationInverseDepleted(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingRotationInverseDepleted(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotationInverseDepleted(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingRotationInverseDepleted(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingRotationInverseDepleted(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingCapacitatedRotationInverseDepleted(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingCapacitatedRotationInverseDepleted(*m_Instance, solution, cost_limit);
    }


    sInt_32 sSMTCBS::find_ShortestNonconflictingCapacitatedRotationInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingCapacitatedRotationInverseDepleted(agent_Paths, cost_limit)) < 0)
	{
	    return cost;
	}
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
	return cost;
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingCapacitatedRotationInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingCapacitatedRotationInverseDepleted(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_ShortestNonconflictingCapacitatedRotationInverseDepleted(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;	
	
	#ifdef sVERBOSE
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();		
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost);
	Context context;	
	
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost + N_agents, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingCapacitatedRotationInverseDepleted(context, instance, agent_Paths, cost)) >= 0)
	    {
		return solution_cost;
	    }

	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}
	return -1;
    }        
    

/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::find_NonconflictingSwapping(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingSwapping(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingSwapping(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;

	instance.construct_SwappingMDD(cost_limit, MDD, extra_cost, extra_MDD);	

	return find_NonconflictingSwapping_GlucoseCollisions(instance, context, MDD, extra_MDD, extra_cost, agent_Paths, cost_limit);
    }


    sInt_32 sSMTCBS::find_NonconflictingSwappingInverse(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingSwappingInverse(context, *m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_NonconflictingSwappingInverse(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;

	instance.construct_SwappingMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);

	return find_NonconflictingSwapping_GlucoseCollisionsInverse(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }


    sInt_32 sSMTCBS::find_NonconflictingSwappingInverseDepleted(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingSwappingInverse(context, *m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_NonconflictingSwappingInverseDepleted(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;

	instance.construct_SwappingMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);

	return find_NonconflictingSwapping_GlucoseCollisionsInverseDepleted(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }        


/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::find_NonconflictingPaths(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPaths(context, *m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_NonconflictingPaths(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;

	instance.construct_PathMDD(cost_limit, MDD, extra_cost, extra_MDD);	

	return find_NonconflictingPaths_GlucoseCollisions(instance, context, MDD, extra_MDD, extra_cost, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_NonconflictingPathsInverse(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPathsInverse(context, *m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_NonconflictingPathsInverse(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;

	instance.construct_PathMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);	

	return find_NonconflictingPaths_GlucoseCollisionsInverse(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }


    sInt_32 sSMTCBS::find_NonconflictingPathsInverseSeparated(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;

	instance.construct_PathMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);

	m_cbs_instance = instance;
	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	m_cbs_instance.collect_Endpoints(source_IDs, goal_IDs);
	m_cbs_instance.m_environment.calc_SourceGoalShortestPaths(m_cbs.m_source_Distances, m_cbs.m_goal_Distances, source_IDs, goal_IDs);

	return find_NonconflictingPaths_GlucoseCollisionsInverseSemiseparated(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }
    

    sInt_32 sSMTCBS::find_NonconflictingPathsInverseDepleted(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;

	instance.construct_PathMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);

	m_cbs_instance = instance;
	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	m_cbs_instance.collect_Endpoints(source_IDs, goal_IDs);
	m_cbs_instance.m_environment.calc_SourceGoalShortestPaths(m_cbs.m_source_Distances, m_cbs.m_goal_Distances, source_IDs, goal_IDs);

	return find_NonconflictingPaths_GlucoseCollisionsInverseDepleted(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }    

    
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::find_NonconflictingPermutation(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPermutation(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingPermutation(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;

	instance.construct_PermutationMDD(cost_limit, MDD, extra_cost, extra_MDD);	

	return find_NonconflictingPermutation_GlucoseCollisions(instance, context, MDD, extra_MDD, extra_cost, agent_Paths, cost_limit);
    }


    sInt_32 sSMTCBS::find_NonconflictingPermutationInverse(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPermutationInverse(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingPermutationInverse(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;
	
	instance.construct_PermutationMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);	

	return find_NonconflictingPermutation_GlucoseCollisionsInverse(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }


    sInt_32 sSMTCBS::find_NonconflictingPermutationInverseDepleted(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPermutationInverseDepleted(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingPermutationInverseDepleted(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;
	
	instance.construct_PermutationMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);	

	return find_NonconflictingPermutation_GlucoseCollisionsInverseDepleted(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }        


/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::find_NonconflictingRotation(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingRotation(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingRotation(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;

	instance.construct_RotationMDD(cost_limit, MDD, extra_cost, extra_MDD);	

	return find_NonconflictingRotation_GlucoseCollisions(instance, context, MDD, extra_MDD, extra_cost, agent_Paths, cost_limit);
    }            

    
    sInt_32 sSMTCBS::find_NonconflictingRotationInverse(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingRotationInverse(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingRotationInverse(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;
	
	instance.construct_RotationMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);	

	return find_NonconflictingRotation_GlucoseCollisionsInverse(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }


    sInt_32 sSMTCBS::find_NonconflictingRotationInverseDepleted(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingRotationInverseDepleted(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingRotationInverseDepleted(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;
	
	instance.construct_RotationMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);	

	return find_NonconflictingRotation_GlucoseCollisionsInverseDepleted(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }

    
    sInt_32 sSMTCBS::find_NonconflictingCapacitatedRotationInverseDepleted(Context &context, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingCapacitatedRotationInverseDepleted(context, *m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sSMTCBS::find_NonconflictingCapacitatedRotationInverseDepleted(Context &context, sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	sInt_32 extra_cost;
	sInstance::MDD_vector MDD, extra_MDD;
	sInstance::InverseMDD_vector inverse_MDD;
	
	instance.construct_RotationMDD(cost_limit, MDD, extra_cost, extra_MDD);
	instance.construct_InverseMDD(MDD, inverse_MDD);	

	return find_NonconflictingCapacitatedRotation_GlucoseCollisionsInverseDepleted(instance, context, MDD, extra_MDD, inverse_MDD, extra_cost, agent_Paths, cost_limit);
    }                

    
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::find_NonconflictingPaths_GlucosePrincipal(const sInstance       &instance,
							       Context               &context,
							       sInstance::MDD_vector &MDD,
							       sInstance::MDD_vector &extra_MDD,
							       sInt_32                extra_cost,
							       AgentPaths_vector     &agent_Paths,
							       sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	sDouble start_time = sStatistics::get_CPU_Seconds();	

	Model sat_Model;
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;

	if (!find_InitialNonconflictingPaths(solver,
					     context,
					     sat_Model,
					     instance,
					     MDD,
					     extra_MDD,
					     extra_cost,
					     cost_limit,
					     agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collision principal_collision;
	
	if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, principal_collision)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPaths(solver,
					      context,
					      sat_Model,
					      principal_collision,
					      instance,
					      MDD,
					      extra_MDD,
					      extra_cost,
					      cost_limit,
					      agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif
	    
	    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, principal_collision)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }

    
    sInt_32 sSMTCBS::find_NonconflictingPaths_GlucoseCollisions(const sInstance       &instance,
								Context               &context,
								sInstance::MDD_vector &MDD,
								sInstance::MDD_vector &extra_MDD,
								sInt_32                extra_cost,
								AgentPaths_vector     &agent_Paths,
								sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Model sat_Model;
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingPaths(solver,
					     context,
					     sat_Model,
					     instance,
					     MDD,
					     extra_MDD,
					     extra_cost,
					     cost_limit,
					     agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	
	if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPaths(solver,
					      context,
					      sat_Model,
					      Collisions,
					      instance,
					      MDD,
					      extra_MDD,
					      extra_cost,
					      cost_limit,
					      agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif
	    
	    Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingPaths_GlucoseCollisionsInverse(const sInstance              &instance,
								       Context                      &context,
								       sInstance::MDD_vector        &MDD,
								       sInstance::MDD_vector        &extra_MDD,
								       sInstance::InverseMDD_vector &inverse_MDD,
								       sInt_32                       extra_cost,
								       AgentPaths_vector            &agent_Paths,
								       sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;
	solver->setIncrementalMode();

	if (!find_InitialNonconflictingPathsInverse(solver,
						    context,
						    sat_Model,
						    instance,
						    MDD,
						    extra_MDD,
						    inverse_MDD,
						    extra_cost,
						    cost_limit,
						    agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	/*
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	*/
	#endif

	Collisions_vector Collisions;
	
	if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    /*
	    Model sat_Model;
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;

	    solver->s_Glucose_timeout = m_timeout;
	    */
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPathsInverse(solver,
						     context,
						     sat_Model,
						     Collisions,
						     instance,
						     MDD,
						     extra_MDD,
						     inverse_MDD,
						     extra_cost,
						     cost_limit,
						     agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    /*
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    */
	    #endif
	    
	    Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingPaths_GlucoseCollisionsInverseSeparated(const sInstance              &instance,
										Context                      &context,
										sInstance::MDD_vector        &MDD,
										sInstance::MDD_vector        &extra_MDD,
										sInstance::InverseMDD_vector &inverse_MDD,
										sInt_32                       extra_cost,
										AgentPaths_vector            &agent_Paths,
										sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;
	solver->setIncrementalMode();

	if (!find_InitialNonconflictingPathsInverse_validity(solver,
							     context,
							     sat_Model,
							     instance,
							     MDD,
							     extra_MDD,
							     inverse_MDD,
							     extra_cost,
							     cost_limit,
							     agent_Paths))   
	{
	    return -1;
	}

	#ifdef sDEBUG	
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	bool model_introduced = false;
	
	if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	{
	    while (true)
	    {
		if (m_timeout >= 0)
		{
		    sDouble end_time = sStatistics::get_CPU_Seconds();
		    if (end_time - start_time > m_timeout)
		    {
			return -2;
		    }
		}
	    
                #ifdef sSTATISTICS
		{
		    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
		}
                #endif

		if (model_introduced)
		{
		    if (!find_NextNonconflictingPathsInverse_validity(solver,
								      context,
								      sat_Model,
								      Collisions,
								      instance,
								      MDD,
								      extra_MDD,
								      inverse_MDD,
								      extra_cost,
								      cost_limit,
								      agent_Paths))
		    {
			return -1;
		    }
		}
		else
		{
		    if (!find_NextNonconflictingPathsInverse_initial(solver,
								     context,
								     sat_Model,
								     Collisions,
								     instance,
								     MDD,
								     extra_MDD,
								     inverse_MDD,
								     extra_cost,
								     cost_limit,
								     agent_Paths))
		    {
			return -1;
		    }
		    model_introduced = true;
		}
		Collisions.clear();		
		
		if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
		{
		    return cummulative;
		}
	    }
	    return -1;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (model_introduced)
	    {
		if (!find_NextNonconflictingPathsInverse_validity(solver,
								 context,
								 sat_Model,
								 Collisions,
								 instance,
								 MDD,
								 extra_MDD,
								 inverse_MDD,
								 extra_cost,
								 cost_limit,
								 agent_Paths))
		{
		    return -1;
		}		
	    }
	    else
	    {
		if (!find_NextNonconflictingPathsInverse_initial(solver,
								 context,
								 sat_Model,
								 Collisions,
								 instance,
								 MDD,
								 extra_MDD,
								 inverse_MDD,
								 extra_cost,
								 cost_limit,
								 agent_Paths))
		{
		    return -1;
		}
		model_introduced = true;
	    }
	    
	    #ifdef sDEBUG
	    /*
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    */
	    #endif
	    
	    Collisions.clear();

	    bool cost_bound_introduced = false;	    
	    
	    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	    {
		while (true)
		{
		    if (m_timeout >= 0)
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			if (end_time - start_time > m_timeout)
			{
			    return -2;
			}
		    }
		    
                    #ifdef sSTATISTICS
		    {
			++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
		    }
                    #endif

		    if (cost_bound_introduced)
		    {
			if (!find_NextNonconflictingPathsInverse_validity(solver,
									  context,
									  sat_Model,
									  Collisions,
									  instance,
									  MDD,
									  extra_MDD,
									  inverse_MDD,
									  extra_cost,
									  cost_limit,
									  agent_Paths))
			{
			    return -1;
			}			
		    }
		    else
		    {
			if (!find_NextNonconflictingPathsInverse_cost(solver,
								      context,
								      sat_Model,
								      Collisions,
								      instance,
								      MDD,
								      extra_MDD,
								      inverse_MDD,
								      extra_cost,
								      cost_limit,
								      agent_Paths))
			{
			    return -1;
			}
			cost_bound_introduced = true;
		    }
		    Collisions.clear();
		    
		    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
		    {
			return cummulative;
		    }
		}
		return -1;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingPaths_GlucoseCollisionsInverseSemiseparated(const sInstance              &instance,
										    Context                      &context,
										    sInstance::MDD_vector        &MDD,
										    sInstance::MDD_vector        &extra_MDD,
										    sInstance::InverseMDD_vector &inverse_MDD,
										    sInt_32                       extra_cost,
										    AgentPaths_vector            &agent_Paths,
										    sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;
	solver->setIncrementalMode();

	if (!find_InitialNonconflictingPathsInverse_validity(solver,
							     context,
							     sat_Model,
							     instance,
							     MDD,
							     extra_MDD,
							     inverse_MDD,
							     extra_cost,
							     cost_limit,
							     agent_Paths))   
	{
	    return -1;
	}

	#ifdef sDEBUG	
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	bool model_introduced = false;
	
	if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	{
	    while (true)
	    {
		if (m_timeout >= 0)
		{
		    sDouble end_time = sStatistics::get_CPU_Seconds();
		    if (end_time - start_time > m_timeout)
		    {
			return -2;
		    }
		}
		
		if (m_timeout >= 0)
		{
		    sDouble end_time = sStatistics::get_CPU_Seconds();
		    if (end_time - start_time > m_timeout)
		    {
			return -2;
		    }
		}
		
                #ifdef sSTATISTICS
		{
		    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
		}
                #endif

		if (model_introduced)
		{
		    if (!find_NextNonconflictingPathsInverse_validity(solver,
								      context,
								      sat_Model,
								      Collisions,
								      instance,
								      MDD,
								      extra_MDD,
								      inverse_MDD,
								      extra_cost,
								      cost_limit,
								      agent_Paths))
		    {
			return -1;
		    }
		}
		else
		{
		    if (!find_NextNonconflictingPathsInverse_initial(solver,
								     context,
								     sat_Model,
								     Collisions,
								     instance,
								     MDD,
								     extra_MDD,
								     inverse_MDD,
								     extra_cost,
								     cost_limit,
								     agent_Paths))
		    {
			return -1;
		    }
		    model_introduced = true;
		}
		Collisions.clear();		
		
		if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
		{
		    return cummulative;
		}
	    }
	    return -1;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (model_introduced)
	    {
		if (!find_NextNonconflictingPathsInverse_validity(solver,
								 context,
								 sat_Model,
								 Collisions,
								 instance,
								 MDD,
								 extra_MDD,
								 inverse_MDD,
								 extra_cost,
								 cost_limit,
								 agent_Paths))
		{
		    return -1;
		}		
	    }
	    else
	    {
		if (!find_NextNonconflictingPathsInverse_initial(solver,
								 context,
								 sat_Model,
								 Collisions,
								 instance,
								 MDD,
								 extra_MDD,
								 inverse_MDD,
								 extra_cost,
								 cost_limit,
								 agent_Paths))
		{
		    return -1;
		}
		model_introduced = true;
	    }
	    
	    #ifdef sDEBUG
	    /*
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    */
	    #endif
	    
	    Collisions.clear();	    
	    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }	
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingPaths_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
									       Context                      &context,
									       sInstance::MDD_vector        &MDD,
									       sInstance::MDD_vector        &extra_MDD,
									       sInstance::InverseMDD_vector &inverse_MDD,
									       sInt_32                       extra_cost,
									       AgentPaths_vector            &agent_Paths,
									       sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;
	solver->setIncrementalMode();

	if (!find_InitialNonconflictingPathsInverseDepleted(solver,
							    context,
							    sat_Model,
							    instance,
							    MDD,
							    extra_MDD,
							    inverse_MDD,
							    extra_cost,
							    cost_limit,
							    agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	/*
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	*/
	#endif

	Collisions_vector Collisions;
	
	if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    /*
	    Model sat_Model;
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;

	    solver->s_Glucose_timeout = m_timeout;
	    */
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPathsInverseDepleted(solver,
							     context,
							     sat_Model,
							     Collisions,
							     instance,
							     MDD,
							     extra_MDD,
							     inverse_MDD,
							     extra_cost,
							     cost_limit,
							     agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    /*
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    */
	    #endif
	    
	    Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingPaths(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }    
    

/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::find_NonconflictingSwapping_GlucosePrincipal(const sInstance       &instance,
								  Context               &context,
								  sInstance::MDD_vector &MDD,
								  sInstance::MDD_vector &extra_MDD,
								  sInt_32                extra_cost,
								  AgentPaths_vector     &agent_Paths,
								  sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingSwapping(solver,
						context,
						sat_Model,
						instance,
						MDD,
						extra_MDD,
						extra_cost,
						cost_limit,
						agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collision principal_collision;
	
	if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, principal_collision)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingSwapping(solver,
						 context,
						 sat_Model,
						 principal_collision,
						 instance,
						 MDD,
						 extra_MDD,
						 extra_cost,
						 cost_limit,
						 agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif
	    
	    if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, principal_collision)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }

    
    sInt_32 sSMTCBS::find_NonconflictingSwapping_GlucoseCollisions(const sInstance       &instance,
								   Context               &context,
								   sInstance::MDD_vector &MDD,
								   sInstance::MDD_vector &extra_MDD,
								   sInt_32                extra_cost,
								   AgentPaths_vector     &agent_Paths,
								   sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingSwapping(solver,
						context,
						sat_Model,
						instance,
						MDD,
						extra_MDD,
						extra_cost,
						cost_limit,
						agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	
	if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingSwapping(solver,
						 context,
						 sat_Model,
						 Collisions,
						 edge_Collisions,
						 instance,
						 MDD,
						 extra_MDD,
						 extra_cost,
						 cost_limit,
						 agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingSwapping_GlucoseCollisionsInverse(const sInstance              &instance,
									  Context                      &context,
									  sInstance::MDD_vector        &MDD,
									  sInstance::MDD_vector        &extra_MDD,
									  sInstance::InverseMDD_vector &inverse_MDD,
									  sInt_32                       extra_cost,
									  AgentPaths_vector            &agent_Paths,
									  sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingSwappingInverse(solver,
						       context,
						       sat_Model,
						       instance,
						       MDD,
						       extra_MDD,
						       inverse_MDD,
						       extra_cost,
						       cost_limit,
						       agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	
	if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingSwappingInverse(solver,
							context,
							sat_Model,
							Collisions,
							edge_Collisions,
							instance,
							MDD,
							extra_MDD,
							inverse_MDD,
							extra_cost,
							cost_limit,
							agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingSwapping_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
										  Context                      &context,
										  sInstance::MDD_vector        &MDD,
										  sInstance::MDD_vector        &extra_MDD,
										  sInstance::InverseMDD_vector &inverse_MDD,
										  sInt_32                       extra_cost,
										  AgentPaths_vector            &agent_Paths,
										  sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingSwappingInverseDepleted(solver,
							       context,
							       sat_Model,
							       instance,
							       MDD,
							       extra_MDD,
							       inverse_MDD,
							       extra_cost,
							       cost_limit,
							       agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	
	if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingSwappingInverseDepleted(solver,
								context,
								sat_Model,
								Collisions,
								edge_Collisions,
								instance,
								MDD,
								extra_MDD,
								inverse_MDD,
								extra_cost,
								cost_limit,
								agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingSwapping(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }            


/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::find_NonconflictingPermutation_GlucosePrincipal(const sInstance       &instance,
								     Context               &context,
								     sInstance::MDD_vector &MDD,
								     sInstance::MDD_vector &extra_MDD,
								     sInt_32                extra_cost,
								     AgentPaths_vector     &agent_Paths,
								     sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	
	
	if (!find_InitialNonconflictingPermutation(solver,
						   context,
						   sat_Model,
						   instance,
						   MDD,
						   extra_MDD,
						   extra_cost,
						   cost_limit,
						   agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collision principal_collision;
	
	if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, principal_collision)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPermutation(solver,
						    context,
						    sat_Model,
						    principal_collision,
						    instance,
						    MDD,
						    extra_MDD,
						    extra_cost,
						    cost_limit,
						    agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif
	    
	    if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, principal_collision)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }

    
    sInt_32 sSMTCBS::find_NonconflictingPermutation_GlucoseCollisions(const sInstance       &instance,
								      Context               &context,
								      sInstance::MDD_vector &MDD,
								      sInstance::MDD_vector &extra_MDD,
								      sInt_32                extra_cost,
								      AgentPaths_vector     &agent_Paths,
								      sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingPermutation(solver,
						   context,
						   sat_Model,
						   instance,
						   MDD,
						   extra_MDD,
						   extra_cost,
						   cost_limit,
						   agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	
	if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPermutation(solver,
						    context,
						    sat_Model,
						    Collisions,
						    instance,
						    MDD,
						    extra_MDD,
						    extra_cost,
						    cost_limit,
						    agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingPermutation_GlucoseCollisionsInverse(const sInstance              &instance,
									     Context                      &context,
									     sInstance::MDD_vector        &MDD,
									     sInstance::MDD_vector        &extra_MDD,
									     sInstance::InverseMDD_vector &inverse_MDD,
									     sInt_32                       extra_cost,
									     AgentPaths_vector            &agent_Paths,
									     sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingPermutationInverse(solver,
							  context,
							  sat_Model,
							  instance,
							  MDD,
							  extra_MDD,
							  inverse_MDD,
							  extra_cost,
							  cost_limit,
							  agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	
	if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPermutationInverse(solver,
							   context,
							   sat_Model,
							   Collisions,
							   instance,
							   MDD,
							   extra_MDD,
							   inverse_MDD,
							   extra_cost,
							   cost_limit,
							   agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingPermutation_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
										     Context                      &context,
										     sInstance::MDD_vector        &MDD,
										     sInstance::MDD_vector        &extra_MDD,
										     sInstance::InverseMDD_vector &inverse_MDD,
										     sInt_32                       extra_cost,
										     AgentPaths_vector            &agent_Paths,
										     sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingPermutationInverseDepleted(solver,
								  context,
								  sat_Model,
								  instance,
								  MDD,
								  extra_MDD,
								  inverse_MDD,
								  extra_cost,
								  cost_limit,
								  agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	
	if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingPermutationInverseDepleted(solver,
								   context,
								   sat_Model,
								   Collisions,
								   instance,
								   MDD,
								   extra_MDD,
								   inverse_MDD,
								   extra_cost,
								   cost_limit,
								   agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingPermutation(instance, agent_Paths, Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }            

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::find_NonconflictingRotation_GlucosePrincipal(const sInstance       &instance,
								  Context               &context,
								  sInstance::MDD_vector &MDD,
								  sInstance::MDD_vector &extra_MDD,
								  sInt_32                extra_cost,
								  AgentPaths_vector     &agent_Paths,
								  sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingRotation(solver,
						context,
						sat_Model,
						instance,
						MDD,
						extra_MDD,
						extra_cost,
						cost_limit,
						agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collision principal_collision;
	
	if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, principal_collision)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingRotation(solver,
						 context,
						 sat_Model,
						 principal_collision,
						 instance,
						 MDD,
						 extra_MDD,
						 extra_cost,
						 cost_limit,
						 agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif
	    
	    if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, principal_collision)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }

    
    sInt_32 sSMTCBS::find_NonconflictingRotation_GlucoseCollisions(const sInstance       &instance,
								   Context               &context,
								   sInstance::MDD_vector &MDD,
								   sInstance::MDD_vector &extra_MDD,
								   sInt_32                extra_cost,
								   AgentPaths_vector     &agent_Paths,
								   sInt_32                cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingRotation(solver,
						context,
						sat_Model,
						instance,
						MDD,
						extra_MDD,
						extra_cost,
						cost_limit,
						agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	
	if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingRotation(solver,
						 context,
						 sat_Model,
						 Collisions,
						 edge_Collisions,
						 instance,
						 MDD,
						 extra_MDD,
						 extra_cost,
						 cost_limit,
						 agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingRotation_GlucoseCollisionsInverse(const sInstance              &instance,
									  Context                      &context,
									  sInstance::MDD_vector        &MDD,
									  sInstance::MDD_vector        &extra_MDD,
									  sInstance::InverseMDD_vector &inverse_MDD,
									  sInt_32                       extra_cost,
									  AgentPaths_vector            &agent_Paths,
									  sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingRotationInverse(solver,
						       context,
						       sat_Model,
						       instance,
						       MDD,
						       extra_MDD,
						       inverse_MDD,
						       extra_cost,
						       cost_limit,
						       agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	
	if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingRotationInverse(solver,
							context,
							sat_Model,
							Collisions,
							edge_Collisions,
							instance,
							MDD,
							extra_MDD,
							inverse_MDD,
							extra_cost,
							cost_limit,
							agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingRotation_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
										  Context                      &context,
										  sInstance::MDD_vector        &MDD,
										  sInstance::MDD_vector        &extra_MDD,
										  sInstance::InverseMDD_vector &inverse_MDD,
										  sInt_32                       extra_cost,
										  AgentPaths_vector            &agent_Paths,
										  sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingRotationInverseDepleted(solver,
							       context,
							       sat_Model,
							       instance,
							       MDD,
							       extra_MDD,
							       inverse_MDD,
							       extra_cost,
							       cost_limit,
							       agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	
	if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingRotationInverseDepleted(solver,
								context,
								sat_Model,
								Collisions,
								edge_Collisions,
								instance,
								MDD,
								extra_MDD,
								inverse_MDD,
								extra_cost,
								cost_limit,
								agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingRotation(instance, agent_Paths, Collisions, edge_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sSMTCBS::find_NonconflictingCapacitatedRotation_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
											     Context                      &context,
											     sInstance::MDD_vector        &MDD,
											     sInstance::MDD_vector        &extra_MDD,
											     sInstance::InverseMDD_vector &inverse_MDD,
											     sInt_32                       extra_cost,
											     AgentPaths_vector            &agent_Paths,
											     sInt_32                       cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	Model sat_Model;
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Glucose::Solver *solver;
	solver = new Glucose::Solver;

	solver->s_Glucose_timeout = m_timeout;	

	if (!find_InitialNonconflictingCapacitatedRotationInverseDepleted(solver,
									  context,
									  sat_Model,
									  instance,
									  MDD,
									  extra_MDD,
									  inverse_MDD,
									  extra_cost,
									  cost_limit,
									  agent_Paths))	   
	{
	    return -1;
	}

	#ifdef sDEBUG
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 agent_path_length = agent_Paths[agent_id].size();
		printf("%d: ", agent_id);
		for (sInt_32 i = 0; i < agent_path_length; ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
	#endif

	Collisions_vector Collisions;
	EdgeCollisions_vector edge_Collisions;
	CapacitatedCollisions_vector capacitated_Collisions;
	
	if ((cummulative = check_NonconflictingCapacitatedRotation(instance,
								   agent_Paths,
								   Collisions,
								   edge_Collisions,
								   capacitated_Collisions)) >= 0)
	{
	    return cummulative;
	}

	while (true)
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    if (!find_NextNonconflictingCapacitatedRotationInverseDepleted(solver,
									   context,
									   sat_Model,
									   Collisions,
									   edge_Collisions,
									   capacitated_Collisions,
									   instance,
									   MDD,
									   extra_MDD,
									   inverse_MDD,
									   extra_cost,
									   cost_limit,
									   agent_Paths))
	    {
		return -1;
	    }
	    
	    #ifdef sDEBUG
	    {
		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    sInt_32 agent_path_length = agent_Paths[agent_id].size();
		    printf("%d: ", agent_id);
		    for (sInt_32 i = 0; i < agent_path_length; ++i)
		    {
			printf("%d ", agent_Paths[agent_id][i]);
		    }
		    printf("\n");
		}
	    }
	    #endif   
	    Collisions.clear();
	    edge_Collisions.clear();
	    capacitated_Collisions.clear();
	    
	    if ((cummulative = check_NonconflictingCapacitatedRotation(instance,
								       agent_Paths,
								       Collisions,
								       edge_Collisions,
								       capacitated_Collisions)) >= 0)
	    {
		return cummulative;
	    }
	}
	return -1;
    }    

    
/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingPaths(Glucose::Solver       *solver,
						  Context               &context,				  
						  Model                 &sat_Model,
						  const sInstance       &instance,
						  sInstance::MDD_vector &MDD,
						  sInstance::MDD_vector &extra_MDD,
						  sInt_32                extra_cost,
						  sInt_32                cost_limit,
						  AgentPaths_vector     &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_PathModelVariables(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraints(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingPathsInverse(Glucose::Solver              *solver,
							 Context                      &context,
							 Model                        &sat_Model,
							 const sInstance              &instance,
							 sInstance::MDD_vector        &MDD,
							 sInstance::MDD_vector        &extra_MDD,
							 sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                       extra_cost,
							 sInt_32                       cost_limit,
							 AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/	
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingPathsInverse_validity(Glucose::Solver              *sUNUSED(solver),
								  Context                      &sUNUSED(context),
								  Model                        &sUNUSED(sat_Model),
								  const sInstance              &instance,
								  sInstance::MDD_vector        &sUNUSED(MDD),
								  sInstance::MDD_vector        &sUNUSED(extra_MDD),
								  sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
								  sInt_32                       extra_cost,
								  sInt_32                       cost_limit,
								  AgentPaths_vector            &agent_Paths) const
    {
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 agent_path_length;
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    Conflicts_vector empty_Conflicts;
	    EdgeConflicts_vector empty_edge_Conflicts;

	    if ((agent_path_length = m_cbs.findStar_NonconflictingSequence(m_cbs_instance.m_environment,
									 m_cbs_instance.m_start_configuration.get_AgentLocation(agent_id),
									 m_cbs_instance.m_goal_configuration.get_AgentLocation(agent_id),
									 cost_limit,
									 extra_cost,
									 empty_Conflicts,
									 empty_edge_Conflicts,					    
									 agent_Paths[agent_id])) < 0)
	    {		
		return false;
	    }
	}
	m_cbs.equalize_NonconflictingSequences(agent_Paths);
	
/*
	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraintsInverse_validity(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
*/
	
	return true;
    }        


    bool sSMTCBS::find_NextNonconflictingPaths(Glucose::Solver       *solver,
					       Context               &context,					       
					       Model                 &sat_Model,
					       const Collision       &principal_collision,
					       const sInstance       &instance,
					       sInstance::MDD_vector &MDD,
					       sInstance::MDD_vector &extra_MDD,
					       sInt_32                extra_cost,
					       sInt_32                sUNUSED(cost_limit),
					       AgentPaths_vector     &agent_Paths) const
    {
	context.m_trans_Collisions.push_back(principal_collision);
	
	refine_PathModelCollision(solver,
				  principal_collision,
				  instance,
				  MDD,
				  extra_MDD,
				  extra_cost,
				  sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingPaths(Glucose::Solver         *solver,
					       Context                 &context,
					       Model                   &sat_Model,
					       const Collisions_vector &Collisions,
					       const sInstance         &instance,
					       sInstance::MDD_vector   &MDD,
					       sInstance::MDD_vector   &extra_MDD,
					       sInt_32                  extra_cost,
					       sInt_32                  sUNUSED(cost_limit),
					       AgentPaths_vector       &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathModelCollisions(solver,
				   Collisions,
				   instance,
				   MDD,
				   extra_MDD,
				   extra_cost,
				   sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingPathsInverse(Glucose::Solver              *solver,
						      Context                      &context,
						      Model                        &sat_Model,
						      const Collisions_vector      &Collisions,
						      const sInstance              &instance,
						      sInstance::MDD_vector        &MDD,
						      sInstance::MDD_vector        &extra_MDD,
						      sInstance::InverseMDD_vector &inverse_MDD,
						      sInt_32                       extra_cost,
						      sInt_32                       sUNUSED(cost_limit),
						      AgentPaths_vector            &agent_Paths) const
    {
	/*
	sInt_32 variable_ID;
	
	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	*/
	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathModelCollisionsInverse(solver,
					  Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  inverse_MDD,
					  extra_cost,
					  sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingPathsInverse_initial(Glucose::Solver              *solver,
							      Context                      &context,
							      Model                        &sat_Model,
							      const Collisions_vector      &Collisions,
							      const sInstance              &instance,
							      sInstance::MDD_vector        &MDD,
							      sInstance::MDD_vector        &extra_MDD,
							      sInstance::InverseMDD_vector &inverse_MDD,
							      sInt_32                       extra_cost,
							      sInt_32                       cost_limit,
							      AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;

	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraintsInverse_validity(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	build_PathModelConstraintsInverse_cost(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);	

	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}		
	refine_PathModelCollisionsInverse(solver,
					  Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  inverse_MDD,
					  extra_cost,
					  sat_Model);	

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}	
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }            

    
    bool sSMTCBS::find_NextNonconflictingPathsInverse_validity(Glucose::Solver              *solver,
							       Context                      &context,
							       Model                        &sat_Model,
							       const Collisions_vector      &Collisions,
							       const sInstance              &instance,
							       sInstance::MDD_vector        &MDD,
							       sInstance::MDD_vector        &extra_MDD,
							       sInstance::InverseMDD_vector &inverse_MDD,
							       sInt_32                       extra_cost,
							       sInt_32                       sUNUSED(cost_limit),
							       AgentPaths_vector            &agent_Paths) const
    {
	/*
	sInt_32 variable_ID;
	
	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	*/
	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathModelCollisionsInverse(solver,
					  Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  inverse_MDD,
					  extra_cost,
					  sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }            


    bool sSMTCBS::find_NextNonconflictingPathsInverse_cost(Glucose::Solver              *solver,
							   Context                      &context,
							   Model                        &sat_Model,
							   const Collisions_vector      &Collisions,
							   const sInstance              &instance,
							   sInstance::MDD_vector        &MDD,
							   sInstance::MDD_vector        &extra_MDD,
							   sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                       extra_cost,
							   sInt_32                       cost_limit,
							   AgentPaths_vector            &agent_Paths) const
    {
	/*
	sInt_32 variable_ID;
	
	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);
	*/

	build_PathModelConstraintsInverse_cost(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathModelCollisionsInverse(solver,
					  Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  inverse_MDD,
					  extra_cost,
					  sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PathModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }        


/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingPathsInverseDepleted(Glucose::Solver              *solver,
								 Context                      &context,
								 Model                        &sat_Model,
								 const sInstance              &instance,
								 sInstance::MDD_vector        &MDD,
								 sInstance::MDD_vector        &extra_MDD,
								 sInstance::InverseMDD_vector &inverse_MDD,
								 sInt_32                       extra_cost,
								 sInt_32                       cost_limit,
								 AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;

	variable_ID = build_PathSmallModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathSmallModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingPathsInverseDepleted_validity(Glucose::Solver              *sUNUSED(solver),
									  Context                      &sUNUSED(context),
									  Model                        &sUNUSED(sat_Model),
									  const sInstance              &instance,
									  sInstance::MDD_vector        &sUNUSED(MDD),
									  sInstance::MDD_vector        &sUNUSED(extra_MDD),
									  sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
									  sInt_32                       extra_cost,
									  sInt_32                       cost_limit,
									  AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 agent_path_length;
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    Conflicts_vector empty_Conflicts;
	    EdgeConflicts_vector empty_edge_Conflicts;

	    if ((agent_path_length = m_cbs.findStar_NonconflictingSequence(m_cbs_instance.m_environment,
									 m_cbs_instance.m_start_configuration.get_AgentLocation(agent_id),
									 m_cbs_instance.m_goal_configuration.get_AgentLocation(agent_id),
									 cost_limit,
									 extra_cost,
									 empty_Conflicts,
									 empty_edge_Conflicts,					    
									 agent_Paths[agent_id])) < 0)
	    {		
		return false;
	    }
	}
	m_cbs.equalize_NonconflictingSequences(agent_Paths);
	
	return true;
    }        


    bool sSMTCBS::find_NextNonconflictingPathsInverseDepleted(Glucose::Solver              *solver,
							      Context                      &context,
							      Model                        &sat_Model,
							      const Collisions_vector      &Collisions,
							      const sInstance              &instance,
							      sInstance::MDD_vector        &MDD,
							      sInstance::MDD_vector        &extra_MDD,
							      sInstance::InverseMDD_vector &inverse_MDD,
							      sInt_32                       extra_cost,
							      sInt_32                       cost_limit,
							      AgentPaths_vector            &agent_Paths) const
    {
	/*
	sInt_32 variable_ID;
	
	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	*/
	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathSmallModelCollisionsInverse(solver,
					       Collisions,
					       instance,
					       MDD,
					       extra_MDD,
					       inverse_MDD,
					       cost_limit,
					       extra_cost,
					       sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingPathsInverseDepleted_initial(Glucose::Solver              *solver,
								      Context                      &context,
								      Model                        &sat_Model,
								      const Collisions_vector      &Collisions,
								      const sInstance              &instance,
								      sInstance::MDD_vector        &MDD,
								      sInstance::MDD_vector        &extra_MDD,
								      sInstance::InverseMDD_vector &inverse_MDD,
								      sInt_32                       extra_cost,
								      sInt_32                       cost_limit,
								      AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;

	variable_ID = build_PathSmallModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PathSmallModelConstraintsInverse_validity(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	build_PathSmallModelConstraintsInverse_cost(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);	

	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}		
	refine_PathSmallModelCollisionsInverse(solver,
					       Collisions,
					       instance,
					       MDD,
					       extra_MDD,
					       inverse_MDD,
					       cost_limit, 
					       extra_cost,
					       sat_Model);	

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}	
	decode_PathSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }            

    
    bool sSMTCBS::find_NextNonconflictingPathsInverseDepleted_validity(Glucose::Solver              *solver,
								       Context                      &context,
								       Model                        &sat_Model,
								       const Collisions_vector      &Collisions,
								       const sInstance              &instance,
								       sInstance::MDD_vector        &MDD,
								       sInstance::MDD_vector        &extra_MDD,
								       sInstance::InverseMDD_vector &inverse_MDD,
								       sInt_32                       extra_cost,
								       sInt_32                       cost_limit,
								       AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathSmallModelCollisionsInverse(solver,
					       Collisions,
					       instance,
					       MDD,
					       extra_MDD,
					       inverse_MDD,
					       cost_limit,
					       extra_cost,
					       sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }            


    bool sSMTCBS::find_NextNonconflictingPathsInverseDepleted_cost(Glucose::Solver              *solver,
								   Context                      &context,
								   Model                        &sat_Model,
								   const Collisions_vector      &Collisions,
								   const sInstance              &instance,
								   sInstance::MDD_vector        &MDD,
								   sInstance::MDD_vector        &extra_MDD,
								   sInstance::InverseMDD_vector &inverse_MDD,
								   sInt_32                       extra_cost,
								   sInt_32                       cost_limit,
								   AgentPaths_vector            &agent_Paths) const
    {
	/*
	sInt_32 variable_ID;
	
	variable_ID = build_PathModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);
	*/

	build_PathSmallModelConstraintsInverse_cost(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PathSmallModelCollisionsInverse(solver,
					       Collisions,
					       instance,
					       MDD,
					       extra_MDD,
					       inverse_MDD,
					       cost_limit,
					       extra_cost,
					       sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PathSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }        

    
/*----------------------------------------------------------------------------*/
    
    
    sInt_32 sSMTCBS::check_NonconflictingPaths(const sInstance         &instance,
					       const AgentPaths_vector &agent_Paths,
					       Collision               &principal_collision) const
    {
	sInt_32 agent_path_length;
	Collision best_collision(sINT_32_MAX, 1, 1, 0, 0);	
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
			    if (*collide_agent > agent_id)
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				
				if (next_collision < best_collision)
				{
				    best_collision = next_collision;
				}
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i - 1);
				sASSERT(i < agent_Paths[*exp_agent].size());

				if (*exp_agent != agent_id)
				{
				    Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
				    if (next_collision < best_collision)
				    {
					best_collision = next_collision;
				    }
				    cummulative = -1;
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < best_collision)
			    {
				best_collision = next_collision;
			    }
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	principal_collision = best_collision;
	
	#ifdef sDEBUG
	{
	    /*
	    if (cummulative < 0)
	    {
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id,
		       principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    }
	    */
	}
	#endif
	
	return cummulative;
    }


    sInt_32 sSMTCBS::check_NonconflictingPaths(const sInstance         &instance,
					       const AgentPaths_vector &agent_Paths,
					       Collisions_vector       &Collisions) const
    {
	sInt_32 agent_path_length;
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	sASSERT(Collisions.empty());
	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
//			    if (*collide_agent > agent_id)
			    if (*collide_agent != agent_id)			    
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				Collisions.push_back(next_collision);
				cummulative = -1;
			    }
			}
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i - 1);
				sASSERT(i < agent_Paths[*exp_agent].size());				
				
				if (*exp_agent != agent_id)
				{
				    Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    Collisions.push_back(next_collision);
				    cummulative = -1;
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
//			if (*collide_agent > agent_id)
			if (*collide_agent != agent_id)			
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			    Collisions.push_back(next_collision);
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	
	#ifdef sDEBUG
	{
	    /*
	    printf("Number of collisions: %ld\n", Collisions.size());
	    for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	    {       	
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       collision->m_agent_A_id, collision->m_level_A, collision->m_vertex_A_id,
		       collision->m_agent_B_id, collision->m_level_B, collision->m_vertex_B_id);
		
		sASSERT(collision->m_agent_A_id != collision->m_agent_B_id);		
	    }
	    */
	}
	#endif
	
	return cummulative;
    }    


/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingSwapping(Glucose::Solver       *solver,
						     Context               &context,						  
						     Model                 &sat_Model,
						     const sInstance       &instance,
						     sInstance::MDD_vector &MDD,
						     sInstance::MDD_vector &extra_MDD,
						     sInt_32                extra_cost,
						     sInt_32                cost_limit,
						     AgentPaths_vector     &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_SwappingModelVariables(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_SwappingModelConstraints(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_SwappingModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingSwappingInverse(Glucose::Solver              *solver,
							    Context                      &context,
							    Model                        &sat_Model,
							    const sInstance              &instance,
							    sInstance::MDD_vector        &MDD,
							    sInstance::MDD_vector        &extra_MDD,
							    sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                       extra_cost,
							    sInt_32                       cost_limit,
							    AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/
	
	variable_ID = build_SwappingModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_SwappingModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_SwappingModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }
    

    bool sSMTCBS::find_NextNonconflictingSwapping(Glucose::Solver       *solver,
						  Context               &context,					       
						  Model                 &sat_Model,
						  const Collision       &principal_collision,
						  const sInstance       &instance,
						  sInstance::MDD_vector &MDD,
						  sInstance::MDD_vector &extra_MDD,
						  sInt_32                extra_cost,
						  sInt_32                cost_limit,
						  AgentPaths_vector     &agent_Paths) const
    {
	context.m_trans_Collisions.push_back(principal_collision);

	refine_SwappingModelCollision(solver,
				      principal_collision,
				      instance,
				      MDD,
				      extra_MDD,
				      cost_limit,
				      extra_cost,
				      sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}	
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_SwappingModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingSwapping(Glucose::Solver             *solver,
						  Context                     &context,
						  Model                       &sat_Model,
						  const Collisions_vector     &Collisions,
						  const EdgeCollisions_vector &edge_Collisions,
						  const sInstance             &instance,
						  sInstance::MDD_vector       &MDD,
						  sInstance::MDD_vector       &extra_MDD,
						  sInt_32                      extra_cost,
						  sInt_32                      cost_limit,
						  AgentPaths_vector           &agent_Paths) const
    {
	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}	
		
	refine_SwappingModelCollisions(solver,
				       Collisions,
				       edge_Collisions,
				       instance,
				       MDD,
				       extra_MDD,
				       cost_limit,
				       extra_cost,
				       sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_SwappingModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingSwappingInverse(Glucose::Solver              *solver,
							 Context                      &context,
							 Model                        &sat_Model,
							 const Collisions_vector      &Collisions,
							 const EdgeCollisions_vector  &edge_Collisions,
							 const sInstance              &instance,
							 sInstance::MDD_vector        &MDD,
							 sInstance::MDD_vector        &extra_MDD,
							 sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                      extra_cost,
							 sInt_32                      cost_limit,
							 AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}	
		
	refine_SwappingModelCollisionsInverse(solver,
					      Collisions,
					      edge_Collisions,
					      instance,
					      MDD,
					      extra_MDD,
					      inverse_MDD,
					      cost_limit,
					      extra_cost,
					      sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_SwappingModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }        


/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingSwappingInverseDepleted(Glucose::Solver              *solver,
								    Context                      &context,
								    Model                        &sat_Model,
								    const sInstance              &instance,
								    sInstance::MDD_vector        &MDD,
								    sInstance::MDD_vector        &extra_MDD,
								    sInstance::InverseMDD_vector &inverse_MDD,
								    sInt_32                       extra_cost,
								    sInt_32                       cost_limit,
								    AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/
	
	variable_ID = build_SwappingSmallModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_SwappingSmallModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_SwappingSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }
    

    bool sSMTCBS::find_NextNonconflictingSwappingInverseDepleted(Glucose::Solver              *solver,
								 Context                      &context,
								 Model                        &sat_Model,
								 const Collisions_vector      &Collisions,
								 const EdgeCollisions_vector  &edge_Collisions,
								 const sInstance              &instance,
								 sInstance::MDD_vector        &MDD,
								 sInstance::MDD_vector        &extra_MDD,
								 sInstance::InverseMDD_vector &inverse_MDD,
								 sInt_32                      extra_cost,
								 sInt_32                      cost_limit,
								 AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}	
		
	refine_SwappingSmallModelCollisionsInverse(solver,
						   Collisions,
						   edge_Collisions,
						   instance,
						   MDD,
						   extra_MDD,
						   inverse_MDD,
						   cost_limit,
						   extra_cost,
						   sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_SwappingSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }        


/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::check_NonconflictingSwapping(const sInstance         &instance,
						  const AgentPaths_vector &agent_Paths,
						  Collision               &principal_collision) const
    {
	sInt_32 agent_path_length;
	Collision best_collision(sINT_32_MAX, 1, 1, 0, 0);	
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
			    if (*collide_agent > agent_id)
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				
				if (next_collision < best_collision)
				{
				    best_collision = next_collision;
				}
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());				

				if (agent_Paths[*exp_agent][ii] != agent_Paths[agent_id][i - 1])
				{				    
				    if (*exp_agent != agent_id)
				    {
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
					if (next_collision < best_collision)
					{
					    best_collision = next_collision;
					}
					cummulative = -1;
				    }
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < best_collision)
			    {
				best_collision = next_collision;
			    }
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	principal_collision = best_collision;
	
	#ifdef sDEBUG
	{
	    /*
	    if (cummulative < 0)
	    {
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id,
		       principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    }
	    */
	}
	#endif
	
	return cummulative;
    }


    sInt_32 sSMTCBS::check_NonconflictingSwapping(const sInstance         &instance,
						  const AgentPaths_vector &agent_Paths,
						  Collisions_vector       &Collisions,
						  EdgeCollisions_vector   &edge_Collisions) const
    {
	sInt_32 agent_path_length;
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	sASSERT(Collisions.empty());
	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
//			    if (*collide_agent > agent_id)
			    if (*collide_agent != agent_id)			    
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				Collisions.push_back(next_collision);
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());				

				if (agent_Paths[*exp_agent][ii] != agent_Paths[agent_id][i - 1])
				{				
				    if (*exp_agent != agent_id)
				    {
					EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									  i-1,
									  agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									  agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					edge_Collisions.push_back(next_edge_collision);
					/*
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
					Collisions.push_back(next_collision);
					*/
					cummulative = -1;
				    }
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		sASSERT(false);
				
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
//			if (*collide_agent > agent_id)
			if (*collide_agent != agent_id)			
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			    Collisions.push_back(next_collision);
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	
	#ifdef sDEBUG
	{
	    /*
	    printf("Number of collisions: %ld\n", Collisions.size());
	    for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	    {       	
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       collision->m_agent_A_id, collision->m_level_A, collision->m_vertex_A_id,
		       collision->m_agent_B_id, collision->m_level_B, collision->m_vertex_B_id);
		
		sASSERT(collision->m_agent_A_id != collision->m_agent_B_id);		
	    }
	    */
	}
	#endif
	
	return cummulative;
    }    

    
/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingPermutation(Glucose::Solver       *solver,
							Context               &context,						  
							Model                 &sat_Model,
							const sInstance       &instance,
							sInstance::MDD_vector &MDD,
							sInstance::MDD_vector &extra_MDD,
							sInt_32                extra_cost,
							sInt_32                cost_limit,
							AgentPaths_vector     &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_PermutationModelVariables(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PermutationModelConstraints(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/	
	decode_PermutationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingPermutationInverse(Glucose::Solver             *solver,
							       Context                      &context,						  
							       Model                        &sat_Model,
							       const sInstance              &instance,
							       sInstance::MDD_vector        &MDD,
							       sInstance::MDD_vector        &extra_MDD,
							       sInstance::InverseMDD_vector &inverse_MDD,
							       sInt_32                       extra_cost,
							       sInt_32                       cost_limit,
							       AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_PermutationModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PermutationModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_PermutationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }    


    bool sSMTCBS::find_NextNonconflictingPermutation(Glucose::Solver       *solver,
						     Context               &context,					       
						     Model                 &sat_Model,
						     const Collision       &principal_collision,
						     const sInstance       &instance,
						     sInstance::MDD_vector &MDD,
						     sInstance::MDD_vector &extra_MDD,
						     sInt_32                extra_cost,
						     sInt_32                cost_limit,
						     AgentPaths_vector     &agent_Paths) const
    {
	context.m_trans_Collisions.push_back(principal_collision);
	
	refine_PermutationModelCollision(solver,
					 principal_collision,
					 instance,
					 MDD,
					 extra_MDD,
					 cost_limit,
					 extra_cost,
					 sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}

/*
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_PermutationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingPermutation(Glucose::Solver         *solver,
						     Context                 &context,
						     Model                   &sat_Model,
						     const Collisions_vector &Collisions,
						     const sInstance         &instance,
						     sInstance::MDD_vector   &MDD,
						     sInstance::MDD_vector   &extra_MDD,
						     sInt_32                  extra_cost,
						     sInt_32                  cost_limit,
						     AgentPaths_vector       &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PermutationModelCollisions(solver,
					  Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  cost_limit,
					  extra_cost,
					  sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PermutationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingPermutationInverse(Glucose::Solver              *solver,
							    Context                      &context,
							    Model                        &sat_Model,
							    const Collisions_vector      &Collisions,
							    const sInstance              &instance,
							    sInstance::MDD_vector        &MDD,
							    sInstance::MDD_vector        &extra_MDD,
							    sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                       extra_cost,
							    sInt_32                       cost_limit,
							    AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PermutationModelCollisionsInverse(solver,
						 Collisions,
						 instance,
						 MDD,
						 extra_MDD,
						 inverse_MDD,
						 cost_limit,
						 extra_cost,
						 sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_PermutationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingPermutationInverseDepleted(Glucose::Solver             *solver,
								       Context                      &context,						  
								       Model                        &sat_Model,
								       const sInstance              &instance,
								       sInstance::MDD_vector        &MDD,
								       sInstance::MDD_vector        &extra_MDD,
								       sInstance::InverseMDD_vector &inverse_MDD,
								       sInt_32                       extra_cost,
								       sInt_32                       cost_limit,
								       AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;

	variable_ID = build_PermutationSmallModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_PermutationSmallModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PermutationSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }    


    bool sSMTCBS::find_NextNonconflictingPermutationInverseDepleted(Glucose::Solver              *solver,
								    Context                      &context,
								    Model                        &sat_Model,
								    const Collisions_vector      &Collisions,
								    const sInstance              &instance,
								    sInstance::MDD_vector        &MDD,
								    sInstance::MDD_vector        &extra_MDD,
								    sInstance::InverseMDD_vector &inverse_MDD,
								    sInt_32                       extra_cost,
								    sInt_32                       cost_limit,
								    AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
		
	refine_PermutationSmallModelCollisionsInverse(solver,
						      Collisions,
						      instance,
						      MDD,
						      extra_MDD,
						      inverse_MDD,
						      cost_limit,
						      extra_cost,
						      sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PermutationSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }        


/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::check_NonconflictingPermutation(const sInstance         &instance,
						     const AgentPaths_vector &agent_Paths,
						     Collision               &principal_collision) const
    {
	sInt_32 agent_path_length;
	Collision best_collision(sINT_32_MAX, 1, 1, 0, 0);	
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
			    if (*collide_agent > agent_id)
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				
				if (next_collision < best_collision)
				{
				    best_collision = next_collision;
				}
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied is acceptable
			{
			    /*
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());

				if (agent_Paths[*exp_agent][ii] != agent_Paths[agent_id][i - 1])
				{				    
				    if (*exp_agent != agent_id)
				    {
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
					if (next_collision < best_collision)
					{
					    best_collision = next_collision;
					}
					cummulative = -1;
				    }
				}
			    }
			    */
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		sASSERT(false);
				
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < best_collision)
			    {
				best_collision = next_collision;
			    }
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	principal_collision = best_collision;
	
	#ifdef sDEBUG
	{
	    /*
	    if (cummulative < 0)
	    {
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id,
		       principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    }
	    */
	}
	#endif
	
	return cummulative;
    }


    sInt_32 sSMTCBS::check_NonconflictingPermutation(const sInstance         &instance,
						     const AgentPaths_vector &agent_Paths,
						     Collisions_vector       &Collisions) const
    {
	sInt_32 agent_path_length;
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	sASSERT(Collisions.empty());
	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
//			    if (*collide_agent > agent_id)
			    if (*collide_agent != agent_id)			    
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				Collisions.push_back(next_collision);
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied is acceptable
			{
			    /*
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());

				if (agent_Paths[*exp_agent][ii] != agent_Paths[agent_id][i - 1])
				{				
				    if (*exp_agent != agent_id)
				    {
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
					Collisions.push_back(next_collision);
					cummulative = -1;
				    }
				}
			    }
			    */
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		sASSERT(false);
				
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
//			if (*collide_agent > agent_id)
			if (*collide_agent != agent_id)			
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			    Collisions.push_back(next_collision);
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	
	#ifdef sDEBUG
	{
	    /*
	    printf("Number of collisions: %ld\n", Collisions.size());
	    for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	    {       	
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       collision->m_agent_A_id, collision->m_level_A, collision->m_vertex_A_id,
		       collision->m_agent_B_id, collision->m_level_B, collision->m_vertex_B_id);
		
		sASSERT(collision->m_agent_A_id != collision->m_agent_B_id);		
	    }
	    */
	}
	#endif
	
	return cummulative;
    }    

    
/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingRotation(Glucose::Solver       *solver,
						     Context               &context,						  
						     Model                 &sat_Model,
						     const sInstance       &instance,
						     sInstance::MDD_vector &MDD,
						     sInstance::MDD_vector &extra_MDD,
						     sInt_32                extra_cost,
						     sInt_32                cost_limit,
						     AgentPaths_vector     &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_RotationModelVariables(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RotationModelConstraints(solver, context, instance, MDD, extra_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/	
	decode_RotationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingRotationInverse(Glucose::Solver              *solver,
							    Context                      &context,						  
							    Model                        &sat_Model,
							    const sInstance              &instance,
							    sInstance::MDD_vector        &MDD,
							    sInstance::MDD_vector        &extra_MDD,
							    sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                       extra_cost,
							    sInt_32                       cost_limit,
							    AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id; agent_id < N_agents; ++agent_id)
	{
	    while (MDD[agent_id].size() > 15)
	    {
		MDD[agent_id].pop_back();
		extra_MDD[agent_id].pop_back();
	    }
	}
*/

	variable_ID = build_RotationModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RotationModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/	
	decode_RotationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }    


    bool sSMTCBS::find_NextNonconflictingRotation(Glucose::Solver       *solver,
						  Context               &context,					       
						  Model                 &sat_Model,
						  const Collision       &principal_collision,
						  const sInstance       &instance,
						  sInstance::MDD_vector &MDD,
						  sInstance::MDD_vector &extra_MDD,
						  sInt_32                extra_cost,
						  sInt_32                cost_limit,
						  AgentPaths_vector     &agent_Paths) const
    {
	context.m_trans_Collisions.push_back(principal_collision);
	
	refine_RotationModelCollision(solver,
				      principal_collision,
				      instance,
				      MDD,
				      extra_MDD,
				      cost_limit,
				      extra_cost,
				      sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif
*/
	decode_RotationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingRotation(Glucose::Solver             *solver,
						  Context                     &context,
						  Model                       &sat_Model,
						  const Collisions_vector     &Collisions,
						  const EdgeCollisions_vector &edge_Collisions,
						  const sInstance             &instance,
						  sInstance::MDD_vector       &MDD,
						  sInstance::MDD_vector       &extra_MDD,
						  sInt_32                      extra_cost,
						  sInt_32                      cost_limit,
						  AgentPaths_vector           &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}	
		
	refine_RotationModelCollisions(solver,
				       Collisions,
				       edge_Collisions,
				       instance,
				       MDD,
				       extra_MDD,
				       cost_limit,
				       extra_cost,
				       sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_RotationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_NextNonconflictingRotationInverse(Glucose::Solver              *solver,
							 Context                      &context,
							 Model                        &sat_Model,
							 const Collisions_vector      &Collisions,
							 const EdgeCollisions_vector  &edge_Collisions,
							 const sInstance              &instance,
							 sInstance::MDD_vector        &MDD,
							 sInstance::MDD_vector        &extra_MDD,
							 sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                       extra_cost,
							 sInt_32                       cost_limit,
							 AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}	
		
	refine_RotationModelCollisionsInverse(solver,
					      Collisions,
					      edge_Collisions,
					      instance,
					      MDD,
					      extra_MDD,
					      inverse_MDD,
					      cost_limit,
					      extra_cost,
					      sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
/*	
	if (!solver->solve())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	    return false;
	}
  	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	}
	#endif	
*/
	decode_RotationModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }
    

/*----------------------------------------------------------------------------*/

    bool sSMTCBS::find_InitialNonconflictingRotationInverseDepleted(Glucose::Solver              *solver,
								    Context                      &context,						  
								    Model                        &sat_Model,
								    const sInstance              &instance,
								    sInstance::MDD_vector        &MDD,
								    sInstance::MDD_vector        &extra_MDD,
								    sInstance::InverseMDD_vector &inverse_MDD,
								    sInt_32                       extra_cost,
								    sInt_32                       cost_limit,
								    AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;

	variable_ID = build_RotationSmallModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RotationSmallModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_RotationSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }    


    bool sSMTCBS::find_NextNonconflictingRotationInverseDepleted(Glucose::Solver              *solver,
								 Context                      &context,
								 Model                        &sat_Model,
								 const Collisions_vector      &Collisions,
								 const EdgeCollisions_vector  &edge_Collisions,
								 const sInstance              &instance,
								 sInstance::MDD_vector        &MDD,
								 sInstance::MDD_vector        &extra_MDD,
								 sInstance::InverseMDD_vector &inverse_MDD,
								 sInt_32                       extra_cost,
								 sInt_32                       cost_limit,
								 AgentPaths_vector            &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}	
		
	refine_RotationSmallModelCollisionsInverse(solver,
						   Collisions,
						   edge_Collisions,
						   instance,
						   MDD,
						   extra_MDD,
						   inverse_MDD,
						   cost_limit,
						   extra_cost,
						   sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_RotationSmallModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }


    bool sSMTCBS::find_InitialNonconflictingCapacitatedRotationInverseDepleted(Glucose::Solver              *solver,
									       Context                      &context,
									       Model                        &sat_Model,
									       const sInstance              &instance,
									       sInstance::MDD_vector        &MDD,
									       sInstance::MDD_vector        &extra_MDD,
									       sInstance::InverseMDD_vector &inverse_MDD,
									       sInt_32                       extra_cost,
									       sInt_32                       cost_limit,
									       AgentPaths_vector            &agent_Paths) const
    {
	sInt_32 variable_ID;

	variable_ID = build_RotationSmallCapacitatedModelVariablesInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RotationSmallCapacitatedModelConstraintsInverse(solver, context, instance, MDD, extra_MDD, inverse_MDD, cost_limit, extra_cost, sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_RotationSmallCapacitatedModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }    


    bool sSMTCBS::find_NextNonconflictingCapacitatedRotationInverseDepleted(Glucose::Solver                    *solver,
									    Context                            &context,
									    Model                              &sat_Model,
									    const Collisions_vector            &Collisions,
									    const EdgeCollisions_vector        &edge_Collisions,
									    const CapacitatedCollisions_vector &capacitated_Collisions,
									    const sInstance                    &instance,
									    sInstance::MDD_vector              &MDD,
									    sInstance::MDD_vector              &extra_MDD,
									    sInstance::InverseMDD_vector       &inverse_MDD,
									    sInt_32                             extra_cost,
									    sInt_32                             cost_limit,
									    AgentPaths_vector                  &agent_Paths) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    context.m_trans_Collisions.push_back(*collision);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    context.m_trans_edge_Collisions.push_back(*edge_collision);
	}
	for (CapacitatedCollisions_vector::const_iterator capacitated_collision = capacitated_Collisions.begin(); capacitated_collision != capacitated_Collisions.end(); ++capacitated_collision)
	{
	    context.m_trans_capacitated_Collisions.push_back(*capacitated_collision);
	}	
			
	refine_RotationSmallCapacitatedModelCollisionsInverse(solver,
							      Collisions,
							      edge_Collisions,
							      capacitated_Collisions,
							      instance,
							      MDD,
							      extra_MDD,
							      inverse_MDD,
							      cost_limit,
							      extra_cost,
							      sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	   
	    return false;
	}

	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_RotationSmallCapacitatedModel(solver, instance, MDD, sat_Model, agent_Paths);
	
	return true;
    }            


/*----------------------------------------------------------------------------*/
    
    sInt_32 sSMTCBS::check_NonconflictingRotation(const sInstance         &instance,
						  const AgentPaths_vector &agent_Paths,
						  Collision               &principal_collision) const
    {
	sInt_32 agent_path_length;
	Collision best_collision(sINT_32_MAX, 1, 1, 0, 0);	
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
			    if (*collide_agent > agent_id)
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				
				if (next_collision < best_collision)
				{
				    best_collision = next_collision;
				}
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());

				if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				{				    
				    if (*exp_agent != agent_id)
				    {
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
					if (next_collision < best_collision)
					{
					    best_collision = next_collision;
					}
					cummulative = -1;
				    }
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		sASSERT(false);
						
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < best_collision)
			    {
				best_collision = next_collision;
			    }
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	principal_collision = best_collision;
	
	#ifdef sDEBUG
	{
	    /*
	    if (cummulative < 0)
	    {
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id,
		       principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    }
	    */
	}
	#endif
	
	return cummulative;
    }


    sInt_32 sSMTCBS::check_NonconflictingRotation(const sInstance         &instance,
						  const AgentPaths_vector &agent_Paths,
						  Collisions_vector       &Collisions,
						  EdgeCollisions_vector   &edge_Collisions) const
    {
	sInt_32 agent_path_length;
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	sASSERT(Collisions.empty());
	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
//			    if (*collide_agent > agent_id)
			    if (*collide_agent != agent_id)			    
			    {
				Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]);
				Collisions.push_back(next_collision);
				cummulative = -1;
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());				

				if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				{				
				    if (*exp_agent != agent_id)
				    {
					EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									  i-1,
									  agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									  agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					edge_Collisions.push_back(next_edge_collision);
					/*
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
					Collisions.push_back(next_collision);
					*/
					cummulative = -1;
				    }
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
//			if (*collide_agent > agent_id)
			if (*collide_agent != agent_id)			
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			    Collisions.push_back(next_collision);
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	
	#ifdef sDEBUG
	{
	    /*
	    printf("Number of collisions: %ld\n", Collisions.size());
	    for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	    {       	
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       collision->m_agent_A_id, collision->m_level_A, collision->m_vertex_A_id,
		       collision->m_agent_B_id, collision->m_level_B, collision->m_vertex_B_id);
		
		sASSERT(collision->m_agent_A_id != collision->m_agent_B_id);		
	    }
	    */
	}
	#endif
	
	return cummulative;
    }    

    
    sInt_32 sSMTCBS::check_NonconflictingCapacitatedRotation(const sInstance         &instance,
							     const AgentPaths_vector &agent_Paths,
							     Collision               &principal_collision,
							     CapacitatedCollision    &principal_capacitated_collision) const
    {
	sInt_32 agent_path_length;
	Collision best_collision(sINT_32_MAX, 1, 1, 0, 0);	
	CapacitatedCollision best_capacitated_collision;
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			if (occupation_collision->second.size() > instance.m_environment.m_Vertices[agent_Paths[agent_id][i]].m_capacity)
			{			    
			    CapacitatedCollision next_capacitated_collision;
			
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				next_capacitated_collision.m_mono_Collisions.push_back(MonoCollision(occupation_collision->second.size(), *collide_agent, i, agent_Paths[agent_id][i]));				    
			    }
			    if (next_capacitated_collision < best_capacitated_collision)
			    {
				best_capacitated_collision = next_capacitated_collision;
			    }
			    cummulative = -1;
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());

				if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				{				    
				    if (*exp_agent != agent_id)
				    {
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
					if (next_collision < best_collision)
					{
					    best_collision = next_collision;
					}
					cummulative = -1;
				    }
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		sASSERT(false);
						
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < best_collision)
			    {
				best_collision = next_collision;
			    }
			    cummulative = -1;			    
			}
		    }		    
		}
	    }
	}
	principal_collision = best_collision;
	principal_capacitated_collision = best_capacitated_collision;
	
	#ifdef sDEBUG
	{
	    /*
	    if (cummulative < 0)
	    {
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id,
		       principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    }
	    */
	}
	#endif
	
	return cummulative;
    }


    sInt_32 sSMTCBS::check_NonconflictingCapacitatedRotation(const sInstance              &instance,
							     const AgentPaths_vector      &agent_Paths,
							     Collisions_vector            &Collisions,
							     EdgeCollisions_vector        &edge_Collisions,
							     CapacitatedCollisions_vector &capacitated_Collisions) const
    {
	sInt_32 agent_path_length;
	    
	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif

	sASSERT(Collisions.empty());
	Cooccupations_vector space_Cooccupations;	
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 cummulative = fill_Cooccupations(instance, agent_Paths, space_Cooccupations);	
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Cooccupations[i].end())
		    {
			if (occupation_collision->second.size() > instance.m_environment.m_Vertices[agent_Paths[agent_id][i]].m_capacity)
			{			    
			    CapacitatedCollision next_capacitated_collision;
			
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				next_capacitated_collision.m_mono_Collisions.push_back(MonoCollision(occupation_collision->second.size(), *collide_agent, i, agent_Paths[agent_id][i]));				    
			    }
			    capacitated_Collisions.push_back(next_capacitated_collision);
			    cummulative = -1;
			}			
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				sASSERT(i < agent_Paths[*exp_agent].size());				

				if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				{				
				    if (*exp_agent != agent_id)
				    {
					EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									  i-1,
									  agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									  agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					edge_Collisions.push_back(next_edge_collision);
					/*
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
					Collisions.push_back(next_collision);
					*/
					cummulative = -1;
				    }
				}
			    }
			}
		    }
		}
	    }
	    if (finished)
	    {
		break;
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    sASSERT(agent_path_length > 0);
	    
	    for (sInt_32 i = agent_path_length; i < space_Cooccupations.size(); ++i)
	    {
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())		    
		{
		    if (occupation_collision->second.size() > instance.m_environment.m_Vertices[agent_Paths[agent_id][agent_path_length - 1]].m_capacity)
		    {			    
			CapacitatedCollision next_capacitated_collision;
			
			for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			{
			    next_capacitated_collision.m_mono_Collisions.push_back(MonoCollision(occupation_collision->second.size(), *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]));
			}
			capacitated_Collisions.push_back(next_capacitated_collision);
			cummulative = -1;
		    }			
		}
	    }
	}
	
	#ifdef sDEBUG
	{
	    /*
	    printf("Number of collisions: %ld\n", Collisions.size());
	    for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	    {       	
		printf("Collision: %d,%d,%d %d,%d,%d\n",
		       collision->m_agent_A_id, collision->m_level_A, collision->m_vertex_A_id,
		       collision->m_agent_B_id, collision->m_level_B, collision->m_vertex_B_id);
		
		sASSERT(collision->m_agent_A_id != collision->m_agent_B_id);		
	    }
	    */
	}
	#endif
	
	return cummulative;
    }    

    
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_PathModelVariables(Glucose::Solver             *sUNUSED(solver),
					      Context                     &sUNUSED(context),
					      const sInstance             &instance,
					      const sInstance::MDD_vector &MDD,
					      const sInstance::MDD_vector &sUNUSED(extra_MDD),
					      sInt_32                      sUNUSED(cost_limit),
					      sInt_32                      sUNUSED(extra_cost),
					      Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }


    sInt_32 sSMTCBS::build_PathModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
						     Context                            &sUNUSED(context),
						     const sInstance                    &instance,
						     const sInstance::MDD_vector        &MDD,
						     const sInstance::MDD_vector        &sUNUSED(extra_MDD),
						     const sInstance::InverseMDD_vector &inverse_MDD,
						     sInt_32                             sUNUSED(cost_limit),						     
						     sInt_32                             sUNUSED(extra_cost),
						     Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    ++N_neighbors;
			}

		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);		    
		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			++N_neighbors;
		    }		    
		    
		    /*
		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    */
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }    


    void sSMTCBS::build_PathModelConstraints(Glucose::Solver             *solver,
					     Context                     &context,					     
					     const sInstance             &instance,
					     const sInstance::MDD_vector &MDD,
					     const sInstance::MDD_vector &extra_MDD,
					     sInt_32                      cost_limit,
					     sInt_32                      extra_cost,
					     Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());

//	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;

		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_target_Identifiers);   
		}
		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					for (sInt_32 vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
								      sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
								      biangular_Identifiers);
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_PathModelCollisions(solver,
 				   context.m_trans_Collisions,
				   instance,
				   MDD,
				   extra_MDD,
				   extra_cost,
				   sat_Model);
    }


    void sSMTCBS::build_PathModelConstraintsInverse(Glucose::Solver                   *solver,
						    Context                            &context,
						    const sInstance                    &instance,
						    const sInstance::MDD_vector        &MDD,
						    const sInstance::MDD_vector        &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             cost_limit,
						    sInt_32                             extra_cost,
						    Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

//	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	/*
	std::vector<VariableIDs_vector> mutex_source_Identifiers;		
	mutex_source_Identifiers.resize(N_vertices);
	*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
/*
		std::vector<VariableIDs_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_vertices);	
*/
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    ++neighbor_index;
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);	
			++neighbor_index;
		    }

		    /*
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    printf("%d\n", MDD[agent_id][layer + 1][v]);
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    */
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);
		}
/*
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    if (mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].size() > 1)
		    {
			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
									  mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		    }
		    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].clear();
		}		
*/
//		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    /
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
	/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    m_solver_Encoder->cast_MultiImpliedImplication(solver,
										   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										   complementary_node_Identifiers, complementary_edge_Identifiers);
				    
				}
				else
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
									     sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
									     biangular_Identifiers);   
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
	*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	
	refine_PathModelCollisionsInverse(solver,
					  context.m_trans_Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  inverse_MDD,
					  extra_cost,
					  sat_Model);
    }    


    void sSMTCBS::build_PathModelConstraintsInverse_validity(Glucose::Solver                   *solver,
							     Context                            &context,
							     const sInstance                    &instance,
							     const sInstance::MDD_vector        &MDD,
							     const sInstance::MDD_vector        &extra_MDD,
							     const sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                             sUNUSED(cost_limit),
							     sInt_32                             extra_cost,
							     Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
//		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
		    }
//		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	
	refine_PathModelCollisionsInverse(solver,
					  context.m_trans_Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  inverse_MDD,
					  extra_cost,
					  sat_Model);
    }


    void sSMTCBS::build_PathModelConstraintsInverse_cost(Glucose::Solver                   *solver,
							 Context                            &sUNUSED(context),
							 const sInstance                    &instance,
							 const sInstance::MDD_vector        &MDD,
							 const sInstance::MDD_vector        &extra_MDD,
							 const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
							 sInt_32                             cost_limit,
							 sInt_32                             extra_cost,
							 Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}	
    }        
    

    void sSMTCBS::refine_PathModelCollision(Glucose::Solver             *solver,
					    const Collision             &collision,
					    const sInstance             &sUNUSED(instance),
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &sUNUSED(extra_MDD),
					    sInt_32                      sUNUSED(extra_cost),
					    Model                       &sat_Model) const
    {
	sInt_32 u = -1, v = -1;
	
	for (u = 0; u < MDD[collision.m_agent_A_id][collision.m_level_A].size(); ++u)
	{
	    if (MDD[collision.m_agent_A_id][collision.m_level_A][u] == collision.m_vertex_A_id)
	    {
		break;
	    }
	}

	for (v = 0; v < MDD[collision.m_agent_B_id][collision.m_level_B].size(); ++v)
	{
	    if (MDD[collision.m_agent_B_id][collision.m_level_B][v] == collision.m_vertex_B_id)
	    {
		break;
	    }	    
	}
	sASSERT(u >= 0 && v >= 0);

	m_solver_Encoder->cast_Mutex(solver,
				     sat_Model.m_vertex_occupancy[collision.m_agent_A_id][collision.m_level_A][u],
				     sat_Model.m_vertex_occupancy[collision.m_agent_B_id][collision.m_level_B][v]);
    }


    void sSMTCBS::refine_PathModelCollisions(Glucose::Solver             *solver,
					     const Collisions_vector     &Collisions,
					     const sInstance             &sUNUSED(instance),
					     const sInstance::MDD_vector &MDD,
					     const sInstance::MDD_vector &sUNUSED(extra_MDD),
					     sInt_32                      sUNUSED(extra_cost),
					     Model                       &sat_Model) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInt_32 u = -1, v = -1;
	
	    for (u = 0; u < MDD[collision->m_agent_A_id][collision->m_level_A].size(); ++u)
	    {
		if (MDD[collision->m_agent_A_id][collision->m_level_A][u] == collision->m_vertex_A_id)
		{
		    break;
		}
	    }
	    
	    for (v = 0; v < MDD[collision->m_agent_B_id][collision->m_level_B].size(); ++v)
	    {
		if (MDD[collision->m_agent_B_id][collision->m_level_B][v] == collision->m_vertex_B_id)
		{
		    break;
		}	    
	    }
	    sASSERT(u >= 0 && v >= 0);

	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}
    }


    void sSMTCBS::refine_PathModelCollisionsInverse(Glucose::Solver                    *solver,
						    const Collisions_vector            &Collisions,
						    const sInstance                    &sUNUSED(instance),
						    const sInstance::MDD_vector        &sUNUSED(MDD),
						    const sInstance::MDD_vector        &sUNUSED(extra_MDD),
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             sUNUSED(extra_cost),
						    Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}	
    }        


    void sSMTCBS::decode_PathModel(Glucose::Solver             *solver,
				   const sInstance             &instance,
				   const sInstance::MDD_vector &MDD,
				   const Model                 &sat_Model,
				   AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    /*
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    */
		    #endif		    
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
	#ifdef sDEBUG
	/*
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 depth = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < depth - 1; ++i)
		{
		    printf("Traversal: %d --> %d ...", agent_Paths[agent_id][i], agent_Paths[agent_id][i + 1]);
		    if (   agent_Paths[agent_id][i] == agent_Paths[agent_id][i + 1]
			|| instance.m_environment.is_Adjacent(agent_Paths[agent_id][i], agent_Paths[agent_id][i + 1]))
		    {
			printf("OK\n");
		    }
		    else
		    {
			printf("no edge\n");
			sASSERT(false);
		    }
		}
	    }	    
	}
	*/
	#endif
    }


/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_PathSmallModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
							  Context                            &sUNUSED(context),
							  const sInstance                    &instance,
							  const sInstance::MDD_vector        &MDD,
							  const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							  const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
							  sInt_32                             sUNUSED(cost_limit),
							  sInt_32                             sUNUSED(extra_cost),
							  Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	/*
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    ++N_neighbors;
			}

		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);		    
		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			++N_neighbors;
		    }		    
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}
        */

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }    


    void sSMTCBS::build_PathSmallModelConstraintsInverse(Glucose::Solver                   *solver,
							 Context                            &context,
							 const sInstance                    &instance,
							 const sInstance::MDD_vector        &MDD,
							 const sInstance::MDD_vector        &extra_MDD,
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                             cost_limit,
							 sInt_32                             extra_cost,
							 Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
			    
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);

			    /*
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			    */
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			/*
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
			*/
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	
	refine_PathSmallModelCollisionsInverse(solver,
					       context.m_trans_Collisions,
					       instance,
					       MDD,
					       extra_MDD,
					       inverse_MDD,
					       cost_limit,
					       extra_cost,
					       sat_Model);
    }

    
    void sSMTCBS::build_PathSmallModelConstraintsInverse_validity(Glucose::Solver                   *solver,
								  Context                            &context,
								  const sInstance                    &instance,
								  const sInstance::MDD_vector        &MDD,
								  const sInstance::MDD_vector        &extra_MDD,
								  const sInstance::InverseMDD_vector &inverse_MDD,
								  sInt_32                             cost_limit,
								  sInt_32                             extra_cost,
								  Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;	

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
			    
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);

			    /*
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			    */
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			/*
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
			*/
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	
	refine_PathSmallModelCollisionsInverse(solver,
					       context.m_trans_Collisions,
					       instance,
					       MDD,
					       extra_MDD,
					       inverse_MDD,
					       cost_limit,
					       extra_cost,
					       sat_Model);
    }   

    
    void sSMTCBS::build_PathSmallModelConstraintsInverse_cost(Glucose::Solver                   *solver,
							      Context                            &sUNUSED(context),
							      const sInstance                    &instance,
							      const sInstance::MDD_vector        &MDD,
							      const sInstance::MDD_vector        &extra_MDD,
							      const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
							      sInt_32                             cost_limit,
							      sInt_32                             extra_cost,
							      Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}	
    }        
    

    void sSMTCBS::refine_PathSmallModelCollisionsInverse(Glucose::Solver                    *solver,
							 const Collisions_vector            &Collisions,
							 const sInstance                    &sUNUSED(instance),
							 const sInstance::MDD_vector        &sUNUSED(MDD),
							 const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                             sUNUSED(cost_limit),
							 sInt_32                             sUNUSED(extra_cost),
							 Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}	
    }        


    void sSMTCBS::decode_PathSmallModel(Glucose::Solver             *solver,
					const sInstance             &instance,
					const sInstance::MDD_vector &MDD,
					const Model                 &sat_Model,
					AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    /*
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    */
		    #endif		    
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
	#ifdef sDEBUG
	/*
	{
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		sInt_32 depth = agent_Paths[agent_id].size();
		for (sInt_32 i = 0; i < depth - 1; ++i)
		{
		    printf("Traversal: %d --> %d ...", agent_Paths[agent_id][i], agent_Paths[agent_id][i + 1]);
		    if (   agent_Paths[agent_id][i] == agent_Paths[agent_id][i + 1]
			|| instance.m_environment.is_Adjacent(agent_Paths[agent_id][i], agent_Paths[agent_id][i + 1]))
		    {
			printf("OK\n");
		    }
		    else
		    {
			printf("no edge\n");
			sASSERT(false);
		    }
		}
	    }	    
	}
	*/
	#endif
    }


/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_SwappingModelVariables(Glucose::Solver             *sUNUSED(solver),
						  Context                     &sUNUSED(context),
						  const sInstance             &instance,
						  const sInstance::MDD_vector &MDD,
						  const sInstance::MDD_vector &sUNUSED(extra_MDD),
						  sInt_32                      sUNUSED(cost_limit),
						  sInt_32                      sUNUSED(extra_cost),
						  Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }


    sInt_32 sSMTCBS::build_SwappingModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
							 Context                            &sUNUSED(context),
							 const sInstance                    &instance,
							 const sInstance::MDD_vector        &MDD,
							 const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                             sUNUSED(cost_limit),
							 sInt_32                             sUNUSED(extra_cost),
							 Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    ++N_neighbors;
			}

		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);		    
		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			++N_neighbors;
		    }		    
		    
		    /*
		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    */
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }
    
    
    void sSMTCBS::build_SwappingModelConstraints(Glucose::Solver             *solver,
						 Context                     &context,					     
						 const sInstance             &instance,
						 const sInstance::MDD_vector &MDD,
						 const sInstance::MDD_vector &extra_MDD,
						 sInt_32                      cost_limit,
						 sInt_32                      extra_cost,
						 Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							sat_Model.m_vertex_occupancy[agent_id][layer][u],
							sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
								    sat_Model.m_layer_cardinality[agent_id][layer],
								    prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		std::vector<VariableIDs_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_vertices);	

		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_target_Identifiers);
		}
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    if (mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].size() > 1)
		    {
			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
									  mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		    }
		}
//		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
	/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    m_solver_Encoder->cast_MultiImpliedImplication(solver,
										   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										   complementary_node_Identifiers, complementary_edge_Identifiers);
				    
				}
				else
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
									     sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
									     biangular_Identifiers);   
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
	*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_SwappingModelCollisions(solver,
				       context.m_trans_Collisions,
				       context.m_trans_edge_Collisions,
				       instance,
				       MDD,
				       extra_MDD,
				       cost_limit,
				       extra_cost,
				       sat_Model);
    }


    void sSMTCBS::build_SwappingModelConstraintsInverse(Glucose::Solver                   *solver,
							Context                            &context,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	std::vector<VariableIDs_vector> mutex_source_Identifiers;		
	mutex_source_Identifiers.resize(N_vertices);		

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
/*
		std::vector<VariableIDs_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_vertices);	
*/
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    ++neighbor_index;
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);	
			++neighbor_index;
		    }

		    /*
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    printf("%d\n", MDD[agent_id][layer + 1][v]);
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    printf("\n");
		    */

		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
/*
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    if (mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].size() > 1)
		    {
			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
									  mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		    }
		    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].clear();
		}
*/		
		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
	/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    m_solver_Encoder->cast_MultiImpliedImplication(solver,
										   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										   complementary_node_Identifiers, complementary_edge_Identifiers);
				    
				}
				else
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
									     sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
									     biangular_Identifiers);   
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
	*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_SwappingModelCollisionsInverse(solver,
					      context.m_trans_Collisions,
					      context.m_trans_edge_Collisions,
					      instance,
					      MDD,
					      extra_MDD,
					      inverse_MDD,
					      cost_limit,
					      extra_cost,
					      sat_Model);
    }    


    void sSMTCBS::refine_SwappingModelCollision(Glucose::Solver             *solver,
						const Collision             &collision,
						const sInstance             &sUNUSED(instance),
						const sInstance::MDD_vector &MDD,
						const sInstance::MDD_vector &sUNUSED(extra_MDD),
						sInt_32                      sUNUSED(cost_limit),
						sInt_32                      sUNUSED(extra_cost),
						Model                       &sat_Model) const
    {
	sInt_32 u = -1, v = -1;
	
	for (u = 0; u < MDD[collision.m_agent_A_id][collision.m_level_A].size(); ++u)
	{
	    if (MDD[collision.m_agent_A_id][collision.m_level_A][u] == collision.m_vertex_A_id)
	    {
		break;
	    }
	}

	for (v = 0; v < MDD[collision.m_agent_B_id][collision.m_level_B].size(); ++v)
	{
	    if (MDD[collision.m_agent_B_id][collision.m_level_B][v] == collision.m_vertex_B_id)
	    {
		break;
	    }	    
	}
	sASSERT(u >= 0 && v >= 0);

	m_solver_Encoder->cast_Mutex(solver,
				     sat_Model.m_vertex_occupancy[collision.m_agent_A_id][collision.m_level_A][u],
				     sat_Model.m_vertex_occupancy[collision.m_agent_B_id][collision.m_level_B][v]);
    }


    void sSMTCBS::refine_SwappingModelCollisions(Glucose::Solver             *solver,
						 const Collisions_vector     &Collisions,
						 const EdgeCollisions_vector &edge_Collisions,
						 const sInstance             &instance,
						 const sInstance::MDD_vector &MDD,
						 const sInstance::MDD_vector &sUNUSED(extra_MDD),
						 sInt_32                      sUNUSED(cost_limit),
						 sInt_32                      sUNUSED(extra_cost),
						 Model                       &sat_Model) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInt_32 u = -1, v = -1;
	
	    for (u = 0; u < MDD[collision->m_agent_A_id][collision->m_level_A].size(); ++u)
	    {
		if (MDD[collision->m_agent_A_id][collision->m_level_A][u] == collision->m_vertex_A_id)
		{
		    break;
		}
	    }
	    
	    for (v = 0; v < MDD[collision->m_agent_B_id][collision->m_level_B].size(); ++v)
	    {
		if (MDD[collision->m_agent_B_id][collision->m_level_B][v] == collision->m_vertex_B_id)
		{
		    break;
		}	    
	    }
	    sASSERT(u >= 0 && v >= 0);

	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    sInt_32 A_u = -1, B_u = -1;
	
	    for (A_u = 0; A_u < MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].size(); ++A_u)
	    {
		if (MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u] == edge_collision->m_edge_A_u_id)
		{
		    break;
		}
	    }
	    for (B_u = 0; B_u < MDD[edge_collision->m_agent_B_id][edge_collision->m_level_A].size(); ++B_u)
	    {
		if (MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u] == edge_collision->m_edge_B_u_id)
		{
		    break;
		}
	    }
	    sASSERT(A_u >= 0 && B_u >= 0);

	    sInt_32 A_n = 0, B_n = 0;

	    for (sInt_32 vv = 0; vv < MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].size(); ++vv)
	    {
		if (   instance.m_environment.is_Adjacent(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u], MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][vv])
		    || MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u] == MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][vv])
		{
		    if (MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][vv] == edge_collision->m_edge_A_v_id)
		    {
			break;
		    }
		    A_n++;
		}
	    }

	    for (sInt_32 vv = 0; vv < MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].size(); ++vv)
	    {
		if (   instance.m_environment.is_Adjacent(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_A][B_u], MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][vv])
		    || MDD[edge_collision->m_agent_B_id][edge_collision->m_level_A][B_u] == MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][vv])
		{
		    if (MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][vv] == edge_collision->m_edge_B_v_id)
		    {
			break;
		    }
		    B_n++;
		}
	    }
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	}	
    }    

    
    void sSMTCBS::refine_SwappingModelCollisionsInverse(Glucose::Solver                    *solver,
							const Collisions_vector            &Collisions,
							const EdgeCollisions_vector        &edge_Collisions,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             sUNUSED(cost_limit),
							sInt_32                             sUNUSED(extra_cost),
							Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}

	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());	    
	    sInt_32 B_u = inverse_B_u->second;	    

	    sInt_32 neighbor_index = 0, A_n = -1, B_n = -1;

	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.end(); ++neighbor)
	    {		
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(neighbor_id);		    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_A_v_id)
		    {
			A_n = neighbor_index;
			break;
		    }
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_A_u_id == edge_collision->m_edge_A_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]);
			
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    {
			A_n = neighbor_index;		    
		    }
		}
	    }
	    sASSERT(A_n != -1);
	    
	    neighbor_index = 0;
    
	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(neighbor_id);
		
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_B_v_id)
		    {
			B_n = neighbor_index;
			break;
		    }		    
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_B_u_id == edge_collision->m_edge_B_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]);
	    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    B_n = neighbor_index;
		}
	    }
	    sASSERT(B_n != -1);
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	}	
    }    

   
    void sSMTCBS::decode_SwappingModel(Glucose::Solver             *solver,
				       const sInstance             &instance,
				       const sInstance::MDD_vector &MDD,
				       const Model                 &sat_Model,
				       AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }

   
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_SwappingSmallModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
							      Context                            &sUNUSED(context),
							      const sInstance                    &instance,
							      const sInstance::MDD_vector        &MDD,
							      const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							      const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
							      sInt_32                             sUNUSED(cost_limit),
							      sInt_32                             sUNUSED(extra_cost),
							      Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }
    

    void sSMTCBS::build_SwappingSmallModelConstraintsInverse(Glucose::Solver                   *solver,
							     Context                            &context,
							     const sInstance                    &instance,
							     const sInstance::MDD_vector        &MDD,
							     const sInstance::MDD_vector        &extra_MDD,
							     const sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                             cost_limit,
							     sInt_32                             extra_cost,
							     Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

//	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}


	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
			    
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);

			    /*
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			    */
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			/*
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
			*/
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
		
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_SwappingSmallModelCollisionsInverse(solver,
						   context.m_trans_Collisions,
						   context.m_trans_edge_Collisions,
						   instance,
						   MDD,
						   extra_MDD,
						   inverse_MDD,
						   cost_limit,
						   extra_cost,
						   sat_Model);
    }    

    
    void sSMTCBS::refine_SwappingSmallModelCollisionsInverse(Glucose::Solver                    *solver,
							     const Collisions_vector            &Collisions,
							     const EdgeCollisions_vector        &edge_Collisions,
							     const sInstance                    &sUNUSED(instance),
							     const sInstance::MDD_vector        &sUNUSED(MDD),
							     const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							     const sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                             sUNUSED(cost_limit),
							     sInt_32                             sUNUSED(extra_cost),
							     Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}

	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    /*
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());	    
	    sInt_32 B_u = inverse_B_u->second;	    

	    sInt_32 neighbor_index = 0, A_n = -1, B_n = -1;

	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.end(); ++neighbor)
	    {		
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(neighbor_id);		    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_A_v_id)
		    {
			A_n = neighbor_index;
			break;
		    }
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_A_u_id == edge_collision->m_edge_A_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]);
			
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    {
			A_n = neighbor_index;		    
		    }
		}
	    }
	    sASSERT(A_n != -1);
	    
	    neighbor_index = 0;
    
	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(neighbor_id);
		
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_B_v_id)
		    {
			B_n = neighbor_index;
			break;
		    }		    
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_B_u_id == edge_collision->m_edge_B_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]);
	    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    B_n = neighbor_index;
		}
	    }
	    sASSERT(B_n != -1);
	    */
	    /*
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	    */

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_v = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(edge_collision->m_edge_A_v_id);
	    sASSERT(inverse_A_v != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_v = inverse_A_v->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());
	    sInt_32 B_u = inverse_B_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_v = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(edge_collision->m_edge_B_v_id);
	    sASSERT(inverse_B_v != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());
	    sInt_32 B_v = inverse_B_v->second;	        
	    	        
	    m_solver_Encoder->cast_4Mutex(solver,
					  sat_Model.m_vertex_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u],
					  sat_Model.m_vertex_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][A_v],
					  sat_Model.m_vertex_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u],
					  sat_Model.m_vertex_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][B_v]);	    
	}	
    }    

   
    void sSMTCBS::decode_SwappingSmallModel(Glucose::Solver             *solver,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const Model                 &sat_Model,
					    AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }

   
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_PermutationModelVariables(Glucose::Solver             *sUNUSED(solver),
						     Context                     &sUNUSED(context),
						     const sInstance             &instance,
						     const sInstance::MDD_vector &MDD,
						     const sInstance::MDD_vector &sUNUSED(extra_MDD),
						     sInt_32                      sUNUSED(cost_limit),
						     sInt_32                      sUNUSED(extra_cost),
						     Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }


    sInt_32 sSMTCBS::build_PermutationModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
							    Context                            &sUNUSED(context),
							    const sInstance                    &instance,
							    const sInstance::MDD_vector        &MDD,
							    const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							    const sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                             sUNUSED(cost_limit),
							    sInt_32                             sUNUSED(extra_cost),
							    Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    ++N_neighbors;
			}

		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);		    
		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			++N_neighbors;
		    }		    
		    
		    /*
		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    */
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }    


    void sSMTCBS::build_PermutationModelConstraints(Glucose::Solver             *solver,
						    Context                     &context,					     
						    const sInstance             &instance,
						    const sInstance::MDD_vector &MDD,
						    const sInstance::MDD_vector &extra_MDD,
						    sInt_32                      cost_limit,
						    sInt_32                      extra_cost,
						    Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							sat_Model.m_vertex_occupancy[agent_id][layer][u],
							sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		std::vector<VariableIDs_vector> mutex_source_Identifiers; 	
		mutex_source_Identifiers.resize(N_vertices);		    						

		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
						     sat_Model.m_vertex_occupancy[agent_id][layer][u],
						     mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
								      mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		}
		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
/*	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    m_solver_Encoder->cast_MultiImpliedImplication(solver,
										   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										   complementary_node_Identifiers, complementary_edge_Identifiers);
				    
				}
				else
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
									     sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
									     biangular_Identifiers);   
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_PermutationModelCollisions(solver,
					  context.m_trans_Collisions,
					  instance,
					  MDD,
					  extra_MDD,
					  cost_limit,
					  extra_cost,
					  sat_Model);
    }


    void sSMTCBS::build_PermutationModelConstraintsInverse(Glucose::Solver                   *solver,
							   Context                            &context,
							   const sInstance                    &instance,
							   const sInstance::MDD_vector        &MDD,
							   const sInstance::MDD_vector        &extra_MDD,
							   const sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                             cost_limit,
							   sInt_32                             extra_cost,
							   Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	std::vector<VariableIDs_vector> mutex_source_Identifiers;		
	mutex_source_Identifiers.resize(N_vertices);		

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
/*
		std::vector<VariableIDs_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_vertices);	
*/
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    ++neighbor_index;
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);	
			++neighbor_index;
		    }

		    /*
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    printf("%d\n", MDD[agent_id][layer + 1][v]);
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    printf("\n");
		    */

		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
		}
/*
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    if (mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].size() > 1)
		    {
			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
									  mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		    }
		    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].clear();
		}
*/		
		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
	/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    m_solver_Encoder->cast_MultiImpliedImplication(solver,
										   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										   complementary_node_Identifiers, complementary_edge_Identifiers);
				    
				}
				else
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
									     sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
									     biangular_Identifiers);   
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
	*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_PermutationModelCollisionsInverse(solver,
						 context.m_trans_Collisions,
						 instance,
						 MDD,
						 extra_MDD,
						 inverse_MDD,
						 cost_limit,
						 extra_cost,
						 sat_Model);
    }        


    void sSMTCBS::refine_PermutationModelCollision(Glucose::Solver             *solver,
						   const Collision             &collision,
						   const sInstance             &sUNUSED(instance),
						   const sInstance::MDD_vector &MDD,
						   const sInstance::MDD_vector &sUNUSED(extra_MDD),
						   sInt_32                      sUNUSED(cost_limit),
						   sInt_32                      sUNUSED(extra_cost),
						   Model                       &sat_Model) const
    {
	sInt_32 u = -1, v = -1;
	
	for (u = 0; u < MDD[collision.m_agent_A_id][collision.m_level_A].size(); ++u)
	{
	    if (MDD[collision.m_agent_A_id][collision.m_level_A][u] == collision.m_vertex_A_id)
	    {
		break;
	    }
	}

	for (v = 0; v < MDD[collision.m_agent_B_id][collision.m_level_B].size(); ++v)
	{
	    if (MDD[collision.m_agent_B_id][collision.m_level_B][v] == collision.m_vertex_B_id)
	    {
		break;
	    }	    
	}
	sASSERT(u >= 0 && v >= 0);

	m_solver_Encoder->cast_Mutex(solver,
				     sat_Model.m_vertex_occupancy[collision.m_agent_A_id][collision.m_level_A][u],
				     sat_Model.m_vertex_occupancy[collision.m_agent_B_id][collision.m_level_B][v]);
    }


    void sSMTCBS::refine_PermutationModelCollisions(Glucose::Solver             *solver,
						    const Collisions_vector     &Collisions,
						    const sInstance             &sUNUSED(instance),
						    const sInstance::MDD_vector &MDD,
						    const sInstance::MDD_vector &sUNUSED(extra_MDD),
						    sInt_32                      sUNUSED(cost_limit),
						    sInt_32                      sUNUSED(extra_cost),
						    Model                       &sat_Model) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInt_32 u = -1, v = -1;
	
	    for (u = 0; u < MDD[collision->m_agent_A_id][collision->m_level_A].size(); ++u)
	    {
		if (MDD[collision->m_agent_A_id][collision->m_level_A][u] == collision->m_vertex_A_id)
		{
		    break;
		}
	    }
	    
	    for (v = 0; v < MDD[collision->m_agent_B_id][collision->m_level_B].size(); ++v)
	    {
		if (MDD[collision->m_agent_B_id][collision->m_level_B][v] == collision->m_vertex_B_id)
		{
		    break;
		}	    
	    }
	    sASSERT(u >= 0 && v >= 0);

	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}
    }    


    void sSMTCBS::refine_PermutationModelCollisionsInverse(Glucose::Solver                    *solver,
							   const Collisions_vector            &Collisions,
							   const sInstance                    &sUNUSED(instance),
							   const sInstance::MDD_vector        &sUNUSED(MDD),
							   const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							   const sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                             sUNUSED(cost_limit),
							   sInt_32                             sUNUSED(extra_cost),
							   Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}
    }        

    
    void sSMTCBS::decode_PermutationModel(Glucose::Solver             *solver,
					  const sInstance             &instance,
					  const sInstance::MDD_vector &MDD,
					  const Model                 &sat_Model,
					  AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }
       

/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_PermutationSmallModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
								 Context                            &sUNUSED(context),
								 const sInstance                    &instance,
								 const sInstance::MDD_vector        &MDD,
								 const sInstance::MDD_vector        &sUNUSED(extra_MDD),
								 const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
								 sInt_32                             sUNUSED(cost_limit),
								 sInt_32                             sUNUSED(extra_cost),
								 Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }    


    void sSMTCBS::build_PermutationSmallModelConstraintsInverse(Glucose::Solver                   *solver,
								Context                            &context,
								const sInstance                    &instance,
								const sInstance::MDD_vector        &MDD,
								const sInstance::MDD_vector        &extra_MDD,
								const sInstance::InverseMDD_vector &inverse_MDD,
								sInt_32                             cost_limit,
								sInt_32                             extra_cost,
								Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

//	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
			    
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);

			    /*
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			    */
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			/*
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
			*/
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	}	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_PermutationModelCollisionsInverse(solver,
						 context.m_trans_Collisions,
						 instance,
						 MDD,
						 extra_MDD,
						 inverse_MDD,
						 cost_limit,
						 extra_cost,
						 sat_Model);
    }        


    void sSMTCBS::refine_PermutationSmallModelCollisionsInverse(Glucose::Solver                    *solver,
								const Collisions_vector            &Collisions,
								const sInstance                    &sUNUSED(instance),
								const sInstance::MDD_vector        &sUNUSED(MDD),
								const sInstance::MDD_vector        &sUNUSED(extra_MDD),
								const sInstance::InverseMDD_vector &inverse_MDD,
								sInt_32                             sUNUSED(cost_limit),
								sInt_32                             sUNUSED(extra_cost),
								Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}
    }        

    
    void sSMTCBS::decode_PermutationSmallModel(Glucose::Solver             *solver,
					       const sInstance             &instance,
					       const sInstance::MDD_vector &MDD,
					       const Model                 &sat_Model,
					       AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }
       

/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_RotationModelVariables(Glucose::Solver             *sUNUSED(solver),
						  Context                     &sUNUSED(context),
						  const sInstance             &instance,
						  const sInstance::MDD_vector &MDD,
						  const sInstance::MDD_vector &sUNUSED(extra_MDD),
						  sInt_32                      sUNUSED(cost_limit),
						  sInt_32                      sUNUSED(extra_cost),
						  Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }


    sInt_32 sSMTCBS::build_RotationModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
							 Context                            &sUNUSED(context),
							 const sInstance                    &instance,
							 const sInstance::MDD_vector        &MDD,
							 const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                             sUNUSED(cost_limit),
							 sInt_32                             sUNUSED(extra_cost),
							 Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}
	sat_Model.m_edge_occupancy.resize(N_agents + 1);

	for (int agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_edge_occupancy[agent_id].resize(N_layers);

	    for (int layer = 0; layer < N_layers; ++layer)
	    {
		sat_Model.m_edge_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());

		for (int u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    ++N_neighbors;
			}

		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);		    
		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			++N_neighbors;
		    }		    
		    
		    /*
		    for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    */
		    sat_Model.m_edge_occupancy[agent_id][layer][u].resize(N_neighbors);

		    for (sInt_32 neighbor = 0; neighbor < N_neighbors; ++neighbor)
		    {
			sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor] = variable_ID++;			
		    }
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }

    
    void sSMTCBS::build_RotationModelConstraints(Glucose::Solver             *solver,
						 Context                     &context,					     
						 const sInstance             &instance,
						 const sInstance::MDD_vector &MDD,
						 const sInstance::MDD_vector &extra_MDD,
						 sInt_32                      cost_limit,
						 sInt_32                      extra_cost,
						 Model                       &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							sat_Model.m_vertex_occupancy[agent_id][layer][u],
							sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
								    sat_Model.m_layer_cardinality[agent_id][layer],
								    prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		std::vector<VariableIDs_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_vertices);		    						

		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
						     sat_Model.m_vertex_occupancy[agent_id][layer][u],
						     mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
								      mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		}
		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    m_solver_Encoder->cast_MultiNegativeImplication(solver,
										    sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										    complementary_edge_Identifiers);
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
*/	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_RotationModelCollisions(solver,
				       context.m_trans_Collisions,
				       context.m_trans_edge_Collisions,
				       instance,
				       MDD,
				       extra_MDD,
				       cost_limit,
				       extra_cost,
				       sat_Model);
    }


    void sSMTCBS::build_RotationModelConstraintsInverse(Glucose::Solver                   *solver,
							Context                            &context,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	std::vector<VariableIDs_vector> mutex_source_Identifiers;		
	mutex_source_Identifiers.resize(N_vertices);		

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
/*
		std::vector<VariableIDs_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_vertices);	
*/
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    sInt_32 neighbor_index = 0;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    ++neighbor_index;
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
//			mutex_source_Identifiers[MDD[agent_id][layer + 1][inverse_neighbor->second]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);	
			++neighbor_index;
		    }

		    /*
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    printf("%d\n", MDD[agent_id][layer + 1][v]);
			    mutex_target_Identifiers.push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);

			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][v]);
//			    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].push_back(sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index]);
			    neighbor_index++;
			}
		    }
		    printf("\n");
		    */

		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
		}
/*
		for (int v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		{
		    if (mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].size() > 1)
		    {
			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver,
									  mutex_source_Identifiers[MDD[agent_id][layer + 1][v]]);
		    }
		    mutex_source_Identifiers[MDD[agent_id][layer + 1][v]].clear();
		}
*/		
		m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	    }
	}
/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
		    
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
	    }
	    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
*/
/*
	for (sInt_32 vertex_id = 0; vertex_id < N_vertices; ++vertex_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		VariableIDs_vector mutex_occupancy_Identifiers;

		for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
		{
		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (MDD[agent_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    m_solver_Encoder->cast_AllMutexConstraint(solver, mutex_occupancy_Identifiers);
		}
	    }
	}
*/
	/*
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{		    
		    sInt_32 neighbor_index = 0;
		    
		    for (sInt_32 v = 0; v < MDD[agent_id][layer + 1].size(); ++v)
		    {
			if (instance.m_environment.is_Adjacent(MDD[agent_id][layer][u], MDD[agent_id][layer + 1][v]) || MDD[agent_id][layer][u] == MDD[agent_id][layer + 1][v])
			{
			    VariableIDs_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
		    
			    if (MDD[agent_id][layer][u] != MDD[agent_id][layer + 1][v])
			    {
				for (sInt_32 other_agent_id = 1; other_agent_id <= N_agents; ++other_agent_id)
				{
				    if (other_agent_id != agent_id)
				    {
					sInt_32 vv = -1;
					
					for (vv = 0; vv < MDD[other_agent_id][layer].size(); ++vv)
					{
					    if (MDD[agent_id][layer + 1][v] == MDD[other_agent_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_agent_id][layer].size())
					{
					    biangular_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_agent_id][layer + 1].size(); ++w)
					    {					    
						if (instance.m_environment.is_Adjacent(MDD[other_agent_id][layer][vv], MDD[other_agent_id][layer + 1][w]) || MDD[other_agent_id][layer][vv] == MDD[other_agent_id][layer + 1][w])
						{
						    if (MDD[other_agent_id][layer][vv] != MDD[other_agent_id][layer + 1][w])
						    {
							if (MDD[other_agent_id][layer + 1][w] == MDD[agent_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sat_Model.m_vertex_occupancy[other_agent_id][layer][vv]);
							    complementary_edge_Identifiers.push_back(sat_Model.m_edge_occupancy[other_agent_id][layer][vv][other_neighbor_index]);
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}					
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    m_solver_Encoder->cast_MultiImpliedImplication(solver,
										   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
										   complementary_node_Identifiers, complementary_edge_Identifiers);
				    
				}
				else
				{
				    m_solver_Encoder->cast_MultiBiangleMutex(solver,
									     sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
									     biangular_Identifiers);   
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
	*/
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_RotationModelCollisionsInverse(solver,
					      context.m_trans_Collisions,
					      context.m_trans_edge_Collisions,
					      instance,
					      MDD,
					      extra_MDD,
					      inverse_MDD,
					      cost_limit,
					      extra_cost,
					      sat_Model);
    }    

    
    void sSMTCBS::refine_RotationModelCollision(Glucose::Solver             *solver,
						const Collision             &collision,
						const sInstance             &sUNUSED(instance),
						const sInstance::MDD_vector &MDD,
						const sInstance::MDD_vector &sUNUSED(extra_MDD),
						sInt_32                      sUNUSED(cost_limit),
						sInt_32                      sUNUSED(extra_cost),
						Model                       &sat_Model) const
    {
	sInt_32 u = -1, v = -1;
	
	for (u = 0; u < MDD[collision.m_agent_A_id][collision.m_level_A].size(); ++u)
	{
	    if (MDD[collision.m_agent_A_id][collision.m_level_A][u] == collision.m_vertex_A_id)
	    {
		break;
	    }
	}

	for (v = 0; v < MDD[collision.m_agent_B_id][collision.m_level_B].size(); ++v)
	{
	    if (MDD[collision.m_agent_B_id][collision.m_level_B][v] == collision.m_vertex_B_id)
	    {
		break;
	    }	    
	}
	sASSERT(u >= 0 && v >= 0);

	m_solver_Encoder->cast_Mutex(solver,
				     sat_Model.m_vertex_occupancy[collision.m_agent_A_id][collision.m_level_A][u],
				     sat_Model.m_vertex_occupancy[collision.m_agent_B_id][collision.m_level_B][v]);
    }


    void sSMTCBS::refine_RotationModelCollisions(Glucose::Solver             *solver,
						 const Collisions_vector     &Collisions,
						 const EdgeCollisions_vector &edge_Collisions,
						 const sInstance             &instance,
						 const sInstance::MDD_vector &MDD,
						 const sInstance::MDD_vector &sUNUSED(extra_MDD),
						 sInt_32                      sUNUSED(cost_limit),
						 sInt_32                      sUNUSED(extra_cost),
						 Model                       &sat_Model) const
    {
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInt_32 u = -1, v = -1;
	
	    for (u = 0; u < MDD[collision->m_agent_A_id][collision->m_level_A].size(); ++u)
	    {
		if (MDD[collision->m_agent_A_id][collision->m_level_A][u] == collision->m_vertex_A_id)
		{
		    break;
		}
	    }
	    
	    for (v = 0; v < MDD[collision->m_agent_B_id][collision->m_level_B].size(); ++v)
	    {
		if (MDD[collision->m_agent_B_id][collision->m_level_B][v] == collision->m_vertex_B_id)
		{
		    break;
		}	    
	    }
	    sASSERT(u >= 0 && v >= 0);

	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    sInt_32 A_u = -1, B_u = -1;
	
	    for (A_u = 0; A_u < MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].size(); ++A_u)
	    {
		if (MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u] == edge_collision->m_edge_A_u_id)
		{
		    break;
		}
	    }
	    for (B_u = 0; B_u < MDD[edge_collision->m_agent_B_id][edge_collision->m_level_A].size(); ++B_u)
	    {
		if (MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u] == edge_collision->m_edge_B_u_id)
		{
		    break;
		}
	    }
	    sASSERT(A_u >= 0 && B_u >= 0);

	    sInt_32 A_n = 0, B_n = 0;

	    for (sInt_32 vv = 0; vv < MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].size(); ++vv)
	    {
		if (   instance.m_environment.is_Adjacent(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u], MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][vv])
		    || MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u] == MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][vv])
		{
		    if (MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][vv] == edge_collision->m_edge_A_v_id)
		    {
			break;
		    }
		    A_n++;
		}
	    }

	    for (sInt_32 vv = 0; vv < MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].size(); ++vv)
	    {
		if (   instance.m_environment.is_Adjacent(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_A][B_u], MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][vv])
		    || MDD[edge_collision->m_agent_B_id][edge_collision->m_level_A][B_u] == MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][vv])
		{
		    if (MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][vv] == edge_collision->m_edge_B_v_id)
		    {
			break;
		    }
		    B_n++;
		}
	    }
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	}	
    }    


    void sSMTCBS::refine_RotationModelCollisionsInverse(Glucose::Solver                    *solver,
							const Collisions_vector            &Collisions,
							const EdgeCollisions_vector        &edge_Collisions,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             sUNUSED(cost_limit),
							sInt_32                             sUNUSED(extra_cost),
							Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}

	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());	    
	    sInt_32 B_u = inverse_B_u->second;	    

	    sInt_32 neighbor_index = 0, A_n = -1, B_n = -1;

	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.end(); ++neighbor)
	    {		
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(neighbor_id);		    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_A_v_id)
		    {
			A_n = neighbor_index;
			break;
		    }
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_A_u_id == edge_collision->m_edge_A_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]);
			
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    {
			A_n = neighbor_index;		    
		    }
		}
	    }
	    sASSERT(A_n != -1);
	    
	    neighbor_index = 0;
    
	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(neighbor_id);
		
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_B_v_id)
		    {
			B_n = neighbor_index;
			break;
		    }		    
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_B_u_id == edge_collision->m_edge_B_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]);
	    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    B_n = neighbor_index;
		}
	    }
	    sASSERT(B_n != -1);
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	}	
    }
    

    void sSMTCBS::decode_RotationModel(Glucose::Solver             *solver,
				       const sInstance             &instance,
				       const sInstance::MDD_vector &MDD,
				       const Model                 &sat_Model,
				       AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }
    
    
/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_RotationSmallModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
							      Context                            &sUNUSED(context),
							      const sInstance                    &instance,
							      const sInstance::MDD_vector        &MDD,
							      const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							      const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
							      sInt_32                             sUNUSED(cost_limit),
							      sInt_32                             sUNUSED(extra_cost),
							      Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }

    
    void sSMTCBS::build_RotationSmallModelConstraintsInverse(Glucose::Solver                   *solver,
							     Context                            &context,
							     const sInstance                    &instance,
							     const sInstance::MDD_vector        &MDD,
							     const sInstance::MDD_vector        &extra_MDD,
							     const sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                             cost_limit,
							     sInt_32                             extra_cost,
							     Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

//	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
			    
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);

			    /*
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			    */
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			/*
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
			*/
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_RotationSmallModelCollisionsInverse(solver,
						   context.m_trans_Collisions,
						   context.m_trans_edge_Collisions,
						   instance,
						   MDD,
						   extra_MDD,
						   inverse_MDD,
						   cost_limit,
						   extra_cost,
						   sat_Model);
    }    


    void sSMTCBS::refine_RotationSmallModelCollisionsInverse(Glucose::Solver                    *solver,
							     const Collisions_vector            &Collisions,
							     const EdgeCollisions_vector        &edge_Collisions,
							     const sInstance                    &sUNUSED(instance),
							     const sInstance::MDD_vector        &sUNUSED(MDD),
							     const sInstance::MDD_vector        &sUNUSED(extra_MDD),
							     const sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                             sUNUSED(cost_limit),
							     sInt_32                             sUNUSED(extra_cost),
							     Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}

	// TODO
	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    /*
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());	    
	    sInt_32 B_u = inverse_B_u->second;	    

	    sInt_32 neighbor_index = 0, A_n = -1, B_n = -1;

	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.end(); ++neighbor)
	    {		
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(neighbor_id);		    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_A_v_id)
		    {
			A_n = neighbor_index;
			break;
		    }
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_A_u_id == edge_collision->m_edge_A_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]);
			
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    {
			A_n = neighbor_index;		    
		    }
		}
	    }
	    sASSERT(A_n != -1);
	    
	    neighbor_index = 0;
    
	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(neighbor_id);
		
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_B_v_id)
		    {
			B_n = neighbor_index;
			break;
		    }		    
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_B_u_id == edge_collision->m_edge_B_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]);
	    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    B_n = neighbor_index;
		}
	    }
	    sASSERT(B_n != -1);	    
	    */
	    /*
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	    */

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_v = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(edge_collision->m_edge_A_v_id);
	    sASSERT(inverse_A_v != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_v = inverse_A_v->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());
	    sInt_32 B_u = inverse_B_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_v = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(edge_collision->m_edge_B_v_id);
	    sASSERT(inverse_B_v != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());
	    sInt_32 B_v = inverse_B_v->second;	        
	    	        
	    m_solver_Encoder->cast_4Mutex(solver,
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u],
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][A_v],
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u],
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][B_v]);
	}	
    }
    

    void sSMTCBS::decode_RotationSmallModel(Glucose::Solver             *solver,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const Model                 &sat_Model,
					    AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    sInt_32 sSMTCBS::build_RotationSmallCapacitatedModelVariablesInverse(Glucose::Solver                    *sUNUSED(solver),
									 Context                            &sUNUSED(context),
									 const sInstance                    &instance,
									 const sInstance::MDD_vector        &MDD,
									 const sInstance::MDD_vector        &sUNUSED(extra_MDD),
									 const sInstance::InverseMDD_vector &sUNUSED(inverse_MDD),
									 sInt_32                             sUNUSED(cost_limit),
									 sInt_32                             sUNUSED(extra_cost),
									 Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());
	
	sInt_32 variable_ID = 1;
	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	sat_Model.m_vertex_occupancy.resize(N_agents + 1);
	sat_Model.m_variable_mapping.push_back(Coordinate());

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_vertex_occupancy[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_vertex_occupancy[agent_id][layer].resize(MDD[agent_id][layer].size());
		for (sInt_32 v = 0; v < MDD[agent_id][layer].size(); ++v)
		{
		    sat_Model.m_vertex_occupancy[agent_id][layer][v] = variable_ID++;
		    sat_Model.m_variable_mapping.push_back(Coordinate(agent_id, MDD[agent_id][layer][v], layer));
		}
	    }
	}

	sat_Model.m_layer_cardinality.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    sat_Model.m_layer_cardinality[agent_id].resize(N_layers + 1);
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		sat_Model.m_layer_cardinality[agent_id][layer] = variable_ID++;
	    }
	}
	
	return variable_ID;
    }

    
    void sSMTCBS::build_RotationSmallCapacitatedModelConstraintsInverse(Glucose::Solver                   *solver,
									Context                            &context,
									const sInstance                    &instance,
									const sInstance::MDD_vector        &MDD,
									const sInstance::MDD_vector        &extra_MDD,
									const sInstance::InverseMDD_vector &inverse_MDD,
									sInt_32                             cost_limit,
									sInt_32                             extra_cost,
									Model                              &sat_Model) const
    {
	sASSERT(!MDD.empty());

//	sInt_32 N_vertices = instance.m_environment.get_VertexCount();
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	sInt_32 N_layers = MDD[1].size() - 1;

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    sASSERT(extra_MDD[agent_id][layer].size() == 1);

		    for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		    {
			if (extra_MDD[agent_id][layer][0] != MDD[agent_id][layer][u])
			{
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_vertex_occupancy[agent_id][layer][u],
							       sat_Model.m_layer_cardinality[agent_id][layer]);
			}
		    }
		    VariableIDs_vector prev_cardinality_Identifiers;

		    for (sInt_32 prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[agent_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][prev_layer]);
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiConjunctiveImplication(solver,
									   sat_Model.m_layer_cardinality[agent_id][layer],
									   prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	VariableIDs_vector cardinality_Identifiers;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 layer = 0; layer <= N_layers; ++layer)
	    {
		if (!extra_MDD[agent_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sat_Model.m_layer_cardinality[agent_id][layer]);
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    if (m_subopt_weight >= 0)
	    {
		if (m_subopt_weight >= 1.0)
		{
		    sDouble sub_extra_cost = cost_limit * m_subopt_weight - (cost_limit - extra_cost);	    
		    m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, sub_extra_cost);		    
		}
	    }
	    else
	    {
		m_solver_Encoder->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    VariableIDs_vector mutex_vertex_Identifiers;
			    
	    for (sInt_32 layer = 0; layer < N_layers; ++layer)
	    {
		VariableIDs_vector mutex_vertex_Identifiers;
		
		for (sInt_32 u = 0; u < MDD[agent_id][layer].size(); ++u)
		{
		    VariableIDs_vector mutex_target_Identifiers;

		    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.begin(); neighbor != instance.m_environment.m_Vertices[MDD[agent_id][layer][u]].m_Neighbors.end(); ++neighbor)
		    {				    
			sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
			sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(neighbor_id);
			
			if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);

			    /*
			    m_solver_Encoder->cast_Implication(solver,
							       sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							       sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			    ++neighbor_index;
			    */
			}
		    }
		    sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[agent_id][layer + 1].find(MDD[agent_id][layer][u]);

		    if (inverse_neighbor != inverse_MDD[agent_id][layer + 1].end())		    
		    {
			mutex_target_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			/*
			m_solver_Encoder->cast_Implication(solver,
							   sat_Model.m_edge_occupancy[agent_id][layer][u][neighbor_index],
							   sat_Model.m_vertex_occupancy[agent_id][layer + 1][inverse_neighbor->second]);
			++neighbor_index;
			*/
		    }
		    mutex_vertex_Identifiers.push_back(sat_Model.m_vertex_occupancy[agent_id][layer][u]);

		    m_solver_Encoder->cast_MultiImplication(solver,
							    sat_Model.m_vertex_occupancy[agent_id][layer][u],
							    mutex_target_Identifiers);
//		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);		    
		}
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][0].size(); ++u)
	    {
		if (MDD[agent_id][0][u] == instance.m_start_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][0][u]);
		}
	    }
	}
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    for (sInt_32 u = 0; u < MDD[agent_id][N_layers].size(); ++u)
	    {
		if (MDD[agent_id][N_layers][u] == instance.m_goal_configuration.get_AgentLocation(agent_id))
		{
		    m_solver_Encoder->cast_BitSet(solver, sat_Model.m_vertex_occupancy[agent_id][N_layers][u]);
		}
	    }
	}
	refine_RotationSmallCapacitatedModelCollisionsInverse(solver,
							      context.m_trans_Collisions,
							      context.m_trans_edge_Collisions,
							      context.m_trans_capacitated_Collisions,
							      instance,
							      MDD,
							      extra_MDD,
							      inverse_MDD,
							      cost_limit,
							      extra_cost,
							      sat_Model);
    }    


    void sSMTCBS::refine_RotationSmallCapacitatedModelCollisionsInverse(Glucose::Solver                    *solver,
									const Collisions_vector            &Collisions,
									const EdgeCollisions_vector        &edge_Collisions,
									const CapacitatedCollisions_vector &capacitated_Collisions,
									const sInstance                    &sUNUSED(instance),
									const sInstance::MDD_vector        &sUNUSED(MDD),
									const sInstance::MDD_vector        &sUNUSED(extra_MDD),
									const sInstance::InverseMDD_vector &inverse_MDD,
									sInt_32                             sUNUSED(cost_limit),
									sInt_32                             sUNUSED(extra_cost),
									Model                              &sat_Model) const
    {	
	for (Collisions_vector::const_iterator collision = Collisions.begin(); collision != Collisions.end(); ++collision)
	{
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_u = inverse_MDD[collision->m_agent_A_id][collision->m_level_A].find(collision->m_vertex_A_id);
	    sASSERT(inverse_u != inverse_MDD[collision->m_agent_A_id][collision->m_level_A].end());
	    sInt_32 u = inverse_u->second;
	    
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[collision->m_agent_B_id][collision->m_level_B].find(collision->m_vertex_B_id);
	    sASSERT(inverse_v != inverse_MDD[collision->m_agent_B_id][collision->m_level_B].end());	    
	    sInt_32 v = inverse_v->second;
	    
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_vertex_occupancy[collision->m_agent_A_id][collision->m_level_A][u],
					 sat_Model.m_vertex_occupancy[collision->m_agent_B_id][collision->m_level_B][v]);
	}

	for (CapacitatedCollisions_vector::const_iterator capacitated_collision = capacitated_Collisions.begin(); capacitated_collision != capacitated_Collisions.end(); ++capacitated_collision)
	{
	    const MonoCollisions_vector &mono_Collisions = capacitated_collision->m_mono_Collisions;	    
	    sBoolEncoder::VariableIDs_vector mono_collision_variable_IDs;

	    for (MonoCollisions_vector::const_iterator mono_collision = mono_Collisions.begin(); mono_collision != mono_Collisions.end(); ++mono_collision)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_v = inverse_MDD[mono_collision->m_agent_id][mono_collision->m_level].find(mono_collision->m_vertex_id);
		sASSERT(inverse_v != inverse_MDD[mono_collision->m_agent_id][mono_collision->m_level].end());	    
		sInt_32 v = inverse_v->second;

		mono_collision_variable_IDs.push_back(sat_Model.m_vertex_occupancy[mono_collision->m_agent_id][mono_collision->m_level][v]);
	    }
	    m_solver_Encoder->cast_CapacityMutex(solver, mono_collision_variable_IDs);
	}

	for (EdgeCollisions_vector::const_iterator edge_collision = edge_Collisions.begin(); edge_collision != edge_Collisions.end(); ++edge_collision)
	{
	    /*
	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());	    
	    sInt_32 B_u = inverse_B_u->second;	    

	    sInt_32 neighbor_index = 0, A_n = -1, B_n = -1;

	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]].m_Neighbors.end(); ++neighbor)
	    {		
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(neighbor_id);		    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_A_v_id)
		    {
			A_n = neighbor_index;
			break;
		    }
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_A_u_id == edge_collision->m_edge_A_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u]);
			
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].end())
		{
		    {
			A_n = neighbor_index;		    
		    }
		}
	    }
	    sASSERT(A_n != -1);
	    
	    neighbor_index = 0;
    
	    for (sVertex::Neighbors_list::const_iterator neighbor = instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.begin();
		 neighbor != instance.m_environment.m_Vertices[MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(neighbor_id);
		
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    if (neighbor_id == edge_collision->m_edge_B_v_id)
		    {
			B_n = neighbor_index;
			break;
		    }		    
		    ++neighbor_index;
		}		
	    }
	    if (edge_collision->m_edge_B_u_id == edge_collision->m_edge_B_v_id)
	    {
		sInstance::InverseVertexIDs_umap::const_iterator inverse_neighbor = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u]);
	    
		if (inverse_neighbor != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].end())
		{
		    B_n = neighbor_index;
		}
	    }
	    sASSERT(B_n != -1);	    
	    */
	    /*
	    m_solver_Encoder->cast_Mutex(solver,
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u][A_n],
					 sat_Model.m_edge_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u][B_n]);
	    */

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_u = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].find(edge_collision->m_edge_A_u_id);
	    sASSERT(inverse_A_u != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_u = inverse_A_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_A_v = inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1].find(edge_collision->m_edge_A_v_id);
	    sASSERT(inverse_A_v != inverse_MDD[edge_collision->m_agent_A_id][edge_collision->m_level_A].end());
	    sInt_32 A_v = inverse_A_v->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_u = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].find(edge_collision->m_edge_B_u_id);
	    sASSERT(inverse_B_u != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());
	    sInt_32 B_u = inverse_B_u->second;

	    sInstance::InverseVertexIDs_umap::const_iterator inverse_B_v = inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1].find(edge_collision->m_edge_B_v_id);
	    sASSERT(inverse_B_v != inverse_MDD[edge_collision->m_agent_B_id][edge_collision->m_level_B].end());
	    sInt_32 B_v = inverse_B_v->second;	        
	    	        
	    m_solver_Encoder->cast_4Mutex(solver,
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A][A_u],
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_A_id][edge_collision->m_level_A + 1][A_v],
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B][B_u],
					 sat_Model.m_vertex_occupancy[edge_collision->m_agent_B_id][edge_collision->m_level_B + 1][B_v]);
	}	
    }
    

    void sSMTCBS::decode_RotationSmallCapacitatedModel(Glucose::Solver             *solver,
						       const sInstance             &instance,
						       const sInstance::MDD_vector &MDD,
						       const Model                 &sat_Model,
						       AgentPaths_vector           &agent_Paths) const
    {
	Configurations_vector mdd_Configurations;
	sInt_32 mdd_depth = MDD[1].size();

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Paths.resize(N_agents + 1);
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_Paths[agent_id].resize(mdd_depth);
	}
	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
		    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		if (variable_ID < sat_Model.m_variable_mapping.size())
		{
		    const Coordinate &coordinate = sat_Model.m_variable_mapping[variable_ID];
		    sInt_32 agent_id = coordinate.m_agent_id;
		    sInt_32 vertex_id = coordinate.m_vertex_id;
		    sInt_32 level = coordinate.m_layer;

		    #ifdef sDEBUG
		    {
			printf("Extratracted from satisfying a:%d, v:%d, l:%d\n", agent_id, level, vertex_id);
		    }
		    #endif
		    agent_Paths[agent_id][level] = vertex_id;
		}
	    }
	}
    }    
    
    
/*----------------------------------------------------------------------------*/
    

} // namespace boOX
