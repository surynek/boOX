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
/* cbs.cpp / 1-224_leibniz                                                    */
/*----------------------------------------------------------------------------*/
//
// Conflict based search implemented in a standard way. A version for MAPF and
// a version for the token swapping have been implemented.
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
#include "core/cbs.h"
#include "util/statistics.h"


using namespace std;
using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{

#ifdef sPROFILE
    static clock_t sequencing_begin, sequencing_end, sequencing_cummul;
    static clock_t revising_begin, revising_end, revising_cummul;
    static clock_t analyzing_begin, analyzing_end, analyzing_cummul;
    static clock_t collecting_begin, collecting_end, collecting_cummul;
#endif


/*----------------------------------------------------------------------------*/
// sCBSBase

    sCBSBase::sCBSBase(sInstance *instance)
	: m_Instance(instance)
	, m_timeout(-1.0)
    {
	// nothing
    }


    sCBSBase::sCBSBase(sInstance *instance, sDouble timeout)
	: m_Instance(instance)
	, m_timeout(timeout)
    {
	// nothing
    }    

    
/*----------------------------------------------------------------------------*/

    sInt_32 sCBSBase::fill_Cooccupations(const sInstance &instance, const AgentPaths_vector &agent_Paths, Cooccupations_vector &space_Cooccupations) const
    {
	sInt_32 agent_path_length;
	sInt_32 cummulative = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Cooccupations.empty())
	    {
		while (space_Cooccupations.size() < agent_path_length)
		{
		    space_Cooccupations.push_back(space_Cooccupations.back());
		}
	    }
	    else
	    {
		space_Cooccupations.resize(agent_path_length);		
	    }
	    space_Cooccupations[0][agent_Paths[agent_id][0]].insert(agent_id);

	    for (sInt_32 i = agent_Paths[agent_id].size() - 2; i >= 0; --i)
	    {
		if (agent_Paths[agent_id][i] != agent_Paths[agent_id][agent_Paths[agent_id].size() - 1])
		{
		    break;
		}
		--agent_path_length;
	    }
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	}		       
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    space_Cooccupations[i][agent_Paths[agent_id][i]].insert(agent_id);
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
		space_Cooccupations[i][agent_Paths[agent_id][agent_path_length - 1]].insert(agent_id);
	    }
	}
	return cummulative;
    }        
    

    void sCBSBase::cast_Occupations(sInt_32 agent_id, sInt_32 prefix_length, const VertexIDs_vector &path, Occupations_vector &space_Occupations) const
    {
	for (sInt_32 i = 0; i < prefix_length; ++i)
	{
	    space_Occupations[i][path[i]] = agent_id;
	}
    }

    
    void sCBSBase::uncast_Occupations(sInt_32 prefix_length, const VertexIDs_vector &path, Occupations_vector &space_Occupations) const
    {
	for (sInt_32 i = 0; i < prefix_length; ++i)
	{
	    space_Occupations[i].erase(path[i]);
	}		
    }


    
    
/*----------------------------------------------------------------------------*/
// sCBS

    sCBS::sCBS(sInstance *instance)
	: sCBSBase(instance)
    {
	// nothing
    }


    sCBS::sCBS(sInstance *instance, sDouble timeout)
	: sCBSBase(instance, timeout)
    {
	// nothing
    }    

    
/*----------------------------------------------------------------------------*/

    sInt_32 sCBS::find_ShortestNonconflictingSwapping(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwapping(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingSwapping(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
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

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingSwapping(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif
	
	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingSwapping(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingSwapping_Delta(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingSwapping_Delta(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingSwapping_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingSwapping_Delta(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingSwapping_Delta(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping_Delta(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingSwapping_Delta(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingSwapping_DeltaStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingSwapping_DeltaStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingSwapping_DeltaStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingSwapping_DeltaStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }


    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingSwapping_DeltaSuperStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingSwapping_DeltaSuperStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingSwapping_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingSwapping_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalSwappingCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TSWAP cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingSwapping_DeltaSuperStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }        
   

    sInt_32 sCBS::find_ShortestNonconflictingPaths(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPaths(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPaths(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPaths(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingPaths(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingPaths_Delta(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPaths_Delta(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPaths_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPaths_Delta(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPaths_Delta(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_Delta(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPaths_Delta(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPaths_DeltaStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPaths_DeltaStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPaths_DeltaStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPaths_DeltaStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPaths_DeltaSuperStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPaths_DeltaSuperStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPaths_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPaths_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPathCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	    
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));
	    }
	    #endif
	    if ((solution_cost = find_NonconflictingPaths_DeltaSuperStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }
    

    sInt_32 sCBS::find_ShortestNonconflictingPermutation(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutation(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPermutation(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingPermutation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingPermutation(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingPermutation_Delta(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPermutation_Delta(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPermutation_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPermutation_Delta(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPermutation_Delta(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation_Delta(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingPermutation_Delta(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPermutation_DeltaStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPermutation_DeltaStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPermutation_DeltaStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingPermutation_DeltaStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }


    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPermutation_DeltaSuperStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingPermutation_DeltaSuperStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingPermutation_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingPermutation_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalPermutationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TPERM cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingPermutation_DeltaSuperStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }        

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation(sSolution &solution, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotation(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingRotation(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const
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

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_ShortestNonconflictingRotation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingRotation(instance, agent_Paths, cost)) >= 0)
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


    sInt_32 sCBS::find_ShortestNonconflictingRotation_Delta(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingRotation_Delta(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingRotation_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingRotation_Delta(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingRotation_Delta(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_Delta(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingRotation_Delta(instance, agent_Paths, cost)) >= 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingRotation_DeltaStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingRotation_DeltaStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingRotation_DeltaStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingRotation_DeltaStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingRotation_DeltaSuperStar(*m_Instance, solution, cost_limit);
    }


    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit)
    {
	sInt_32 cost;
	AgentPaths_vector agent_Paths;

	if ((cost = find_ShortestNonconflictingRotation_DeltaSuperStar(agent_Paths, cost_limit)) < 0)
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

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_ShortestNonconflictingRotation_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_ShortestNonconflictingRotation_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	sInt_32 solution_cost, max_individual_cost;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
        #ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	sInt_32 min_total_cost = instance.estimate_TotalRotationCost(max_individual_cost) + N_agents;
	
//	for (sInt_32 cost = 0; cost <= cost_limit; ++cost)
	sInt_32 extra = 0;
	for (sInt_32 cost = min_total_cost; cost <= cost_limit; ++cost)	
	{
	    m_delta_conflict_node_IDs.clear();
	    m_delta_path_node_IDs.clear();
	    m_delta_agent_Conflicts.clear();
	    m_delta_agent_edge_Conflicts.clear();
	    m_first_agent_Paths.clear();
	    m_delta_agent_Paths.clear();
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving TROT cost %d (elapsed time [seconds]: %.3f)...\n", cost, (end_time - start_time));		
	    }
	    #endif	    
	    if ((solution_cost = find_NonconflictingRotation_DeltaSuperStar(instance, agent_Paths, cost, extra)) >= 0)
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
	    ++extra;
	}
	return -1;
    }        


/*----------------------------------------------------------------------------*/
        
    sInt_32 sCBS::find_NonconflictingSwapping(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingSwapping(*m_Instance, agent_Paths, cost_limit);
    }
    
    
    sInt_32 sCBS::find_NonconflictingSwapping(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

//	return find_NonconflictingSwapping_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingSwapping_principalCollision(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);
    }


    sInt_32 sCBS::find_NonconflictingSwapping_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_NonconflictingSwapping_Delta(*m_Instance, agent_Paths, cost_limit);
    }
    
   
    sInt_32 sCBS::find_NonconflictingSwapping_Delta(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);
	
	return find_NonconflictingSwapping_principalCollision_Delta(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);		
    }


    sInt_32 sCBS::find_NonconflictingSwapping_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingSwapping_DeltaStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }
    
   
    sInt_32 sCBS::find_NonconflictingSwapping_DeltaStar(sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	
	return find_NonconflictingSwapping_principalCollision_DeltaStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }

    
    sInt_32 sCBS::find_NonconflictingSwapping_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingSwapping_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }
    
   
    sInt_32 sCBS::find_NonconflictingSwapping_DeltaSuperStar(sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	
	return find_NonconflictingSwapping_principalCollision_DeltaSuperStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }    

    
    sInt_32 sCBS::find_NonconflictingPaths(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPaths(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_NonconflictingPaths(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);	
//	return find_NonconflictingPaths_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingPaths_principalCollision(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);
    }


    sInt_32 sCBS::find_NonconflictingPaths_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_NonconflictingPaths_Delta(*m_Instance, agent_Paths, cost_limit);
    }
    
   
    sInt_32 sCBS::find_NonconflictingPaths_Delta(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);
	
	return find_NonconflictingPaths_principalCollision_Delta(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);		
    }


    sInt_32 sCBS::find_NonconflictingPaths_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingPaths_DeltaStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }
    
   
    sInt_32 sCBS::find_NonconflictingPaths_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	
	return find_NonconflictingPaths_principalCollision_DeltaStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }        


    sInt_32 sCBS::find_NonconflictingPaths_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingPaths_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }
    
   
    sInt_32 sCBS::find_NonconflictingPaths_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	
	return find_NonconflictingPaths_principalCollision_DeltaSuperStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }
    
    
    sInt_32 sCBS::find_NonconflictingPermutation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingPermutation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_NonconflictingPermutation(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);		
//	return find_NonconflictingPermutation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingPermutation_principalCollision(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);
    }


    sInt_32 sCBS::find_NonconflictingPermutation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_NonconflictingPermutation_Delta(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_NonconflictingPermutation_Delta(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);
	
//	return find_NonconflictingPermutation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingPermutation_principalCollision_Delta(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);
    }


    sInt_32 sCBS::find_NonconflictingPermutation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingPermutation_DeltaStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }

    
    sInt_32 sCBS::find_NonconflictingPermutation_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	
//	return find_NonconflictingPermutation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingPermutation_principalCollision_DeltaStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }


    sInt_32 sCBS::find_NonconflictingPermutation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingPermutation_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }

    
    sInt_32 sCBS::find_NonconflictingPermutation_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;	
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	
//	return find_NonconflictingPermutation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingPermutation_principalCollision_DeltaSuperStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }            


    sInt_32 sCBS::find_NonconflictingRotation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	return find_NonconflictingRotation(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_NonconflictingRotation(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);	
//	return find_NonconflictingRotation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingRotation_principalCollision(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);
    }


    sInt_32 sCBS::find_NonconflictingRotation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	return find_NonconflictingRotation_Delta(*m_Instance, agent_Paths, cost_limit);
    }

    
    sInt_32 sCBS::find_NonconflictingRotation_Delta(const sInstance  &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);
	
//	return find_NonconflictingRotation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingRotation_principalCollision_Delta(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit);
    }


    sInt_32 sCBS::find_NonconflictingRotation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingRotation_DeltaStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }

    
    sInt_32 sCBS::find_NonconflictingRotation_DeltaStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);	
	
//	return find_NonconflictingRotation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingRotation_principalCollision_DeltaStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }


    sInt_32 sCBS::find_NonconflictingRotation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	return find_NonconflictingRotation_DeltaSuperStar(*m_Instance, agent_Paths, cost_limit, extra_cost);
    }

    
    sInt_32 sCBS::find_NonconflictingRotation_DeltaSuperStar(sInstance &instance, AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost)
    {
	AgentConflicts_vector agent_Conflicts;
	AgentEdgeConflicts_vector agent_edge_Conflicts;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	agent_Conflicts.resize(N_agents + 1);
	agent_edge_Conflicts.resize(N_agents + 1);

	m_delta_conflict_node_IDs.resize(N_agents + 1, -1);
	m_delta_path_node_IDs.resize(N_agents + 1, -1);	
	m_delta_agent_Conflicts.resize(N_agents + 1);
	m_delta_agent_edge_Conflicts.resize(N_agents + 1);
	m_first_agent_Paths.resize(N_agents + 1);	
	m_delta_agent_Paths.resize(N_agents + 1);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	instance.collect_Endpoints(source_IDs, goal_IDs);
	instance.m_environment.calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);	
	
//	return find_NonconflictingRotation_prioritizedCooccupation(instance, agent_Conflicts, agent_Paths, cost_limit);
	return find_NonconflictingRotation_principalCollision_DeltaSuperStar(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit, extra_cost);
    }        

    
/*----------------------------------------------------------------------------*/

    #define sCBS_ADD_AGENT_CONFLICT(agent_id, step, vertex_id)    \
    {                                                             \
	if (agent_Conflicts[(agent_id)].size() <= (step))         \
	{                                                         \
	    agent_Conflicts[(agent_id)].resize((step) + 1);       \
	}                                                         \
	agent_Conflicts[(agent_id)][(step)].insert((vertex_id));  \
    }

    
    #define sCBS_DEL_AGENT_CONFLICT(agent_id, step, vertex_id)    \
    {                                                             \
	agent_Conflicts[(agent_id)][(step)].erase((vertex_id));   \
    }


    #define sCBS_ADD_AGENT_EDGE_CONFLICT(agent_id, step, edge_u_id, edge_v_id)     \
    {                                                                              \
	if (agent_edge_Conflicts[(agent_id)].size() <= (step))                     \
	{                                                                          \
	    agent_edge_Conflicts[(agent_id)].resize((step) + 1);                   \
	}                                                                          \
	agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].insert((edge_v_id)); \
    }

    
    #define sCBS_DEL_AGENT_CONFLICT(agent_id, step, vertex_id)    \
    {                                                             \
	agent_Conflicts[(agent_id)][(step)].erase((vertex_id));   \
    }

    
    #define sCBS_DEL_AGENT_EDGE_CONFLICT(agent_id, step, edge_u_id, edge_v_id)     \
    {                                                                              \
	agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].erase((edge_v_id));  \
	if (agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].empty())         \
        {                                                                          \
	    agent_edge_Conflicts[(agent_id)][(step)].erase((edge_u_id));           \
	}                                                                          \
    }        


    #define sCBS_ADD_NODE_AGENT_CONFLICT(node, agent_id, step, vertex_id)  \
    {                                                                      \
	if ((node).m_agent_Conflicts[(agent_id)].size() <= (step))         \
	{                                                                  \
	    (node).m_agent_Conflicts[(agent_id)].resize((step) + 1);       \
	}                                                                  \
	(node).m_agent_Conflicts[(agent_id)][(step)].insert((vertex_id));  \
    }


    #define sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(node, agent_id, step, edge_u_id, edge_v_id)    \
    {                                                                                        \
	if ((node).m_agent_edge_Conflicts[(agent_id)].size() <= (step))                      \
	{                                                                                    \
	    (node).m_agent_edge_Conflicts[(agent_id)].resize((step) + 1);                    \
	}                                                                                    \
	(node).m_agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].insert((edge_v_id));  \
    }    

    
    #define sCBS_DEL_NODE_AGENT_CONFLICT(node, agent_id, step, vertex_id)  \
    {                                                                      \
	node.m_agent_Conflicts[(agent_id)][(step)].erase((vertex_id));     \
    }


    #define sCBS_DEL_NODE_AGENT_EDGE_CONFLICT(node, agent_id, step, edge_u_id, edge_v_id)  \
    {                                                                                      \
        (node).agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].erase((vertex_id));   \
	if ((node).agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].empty())          \
        {                                                                                  \
	    (node).agent_edge_Conflicts[(agent_id)][(step)].erase((edge_u_id));            \
	}                                                                                  \
    }


    #define sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(node, agent_id, step, edge_u_id, edge_v_id)    \
    {                                                                                        \
	if ((node).m_agent_edge_Conflicts[(agent_id)].size() <= (step))                      \
	{                                                                                    \
	    (node).m_agent_edge_Conflicts[(agent_id)].resize((step) + 1);                    \
	}                                                                                    \
	(node).m_agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].insert((edge_v_id));  \
    }    

    
    #define sCBS_DEL_NODE_AGENT_CONFLICT(node, agent_id, step, vertex_id)  \
    {                                                                      \
	node.m_agent_Conflicts[(agent_id)][(step)].erase((vertex_id));     \
    }


    #define sCBS_DEL_NODE_AGENT_EDGE_CONFLICT(node, agent_id, step, edge_u_id, edge_v_id)  \
    {                                                                                      \
        (node).agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].erase((vertex_id));   \
	if ((node).agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].empty())          \
        {                                                                                  \
	    (node).agent_edge_Conflicts[(agent_id)][(step)].erase((edge_u_id));            \
	}                                                                                  \
    }        


    #define sCBS_ADD_AGENT_DELTA_CONFLICT(agent_id, step, vertex_id)      \
    {                                                                     \
	if (m_delta_agent_Conflicts[(agent_id)].size() <= (step))         \
	{                                                                 \
	    m_delta_agent_Conflicts[(agent_id)].resize((step) + 1);       \
	}                                                                 \
	m_delta_agent_Conflicts[(agent_id)][(step)].insert((vertex_id));  \
    }

    
    #define sCBS_DEL_AGENT_DELTA_CONFLICT(agent_id, step, vertex_id)     \
    {                                                                    \
	m_delta_agent_Conflicts[(agent_id)][(step)].erase((vertex_id));  \
    }


    #define sCBS_ADD_AGENT_DELTA_EDGE_CONFLICT(agent_id, step, edge_u_id, edge_v_id)       \
    {                                                                                      \
	if (m_delta_agent_edge_Conflicts[(agent_id)].size() <= (step))                     \
	{                                                                                  \
	    m_delta_agent_edge_Conflicts[(agent_id)].resize((step) + 1);                   \
	}                                                                                  \
	m_delta_agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].insert((edge_v_id)); \
    }

    
    #define sCBS_DEL_AGENT_DELTA_CONFLICT(agent_id, step, vertex_id)    \
    {                                                                   \
	m_delta_agent_Conflicts[(agent_id)][(step)].erase((vertex_id)); \
    }

    
    #define sCBS_DEL_AGENT_DELTA_EDGE_CONFLICT(agent_id, step, edge_u_id, edge_v_id)       \
    {                                                                                      \
	m_delta_agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].erase((edge_v_id));  \
	if (m_delta_agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].empty())         \
        {                                                                                  \
	    m_delta_agent_edge_Conflicts[(agent_id)][(step)].erase((edge_u_id));           \
	}                                                                                  \
    }        


    #define sCBS_SHOW_CONFLICTS(agent_Conflicts)			                          	 \
    {                                                                                                    \
	printf("Vertex conflicts\n");					                                 \
	for (sInt_32 agent_id = 1; agent_id < agent_Conflicts.size(); ++agent_id)                        \
	{                                                                                                \
	    printf("Agent: %d\n", agent_id);                                                             \
	    sInt_32 N_steps = agent_Conflicts[agent_id].size();	                                         \
	    for (sInt_32 step = 0; step < N_steps; ++step)                                               \
	    {                                                                                            \
		printf("  %d: ", step);                                                                  \
		for (VertexIDs_uset::const_iterator conflict = agent_Conflicts[agent_id][step].begin();  \
		     conflict != agent_Conflicts[agent_id][step].end(); ++conflict)                      \
		{                                                                                        \
		    printf("%d ", *conflict);                                                            \
		}                                                                                        \
		printf("\n");                                                                            \
	    }                                                                                            \
	}                                                                                                \
    }


    #define sCBS_SHOW_DELTA_CONFLICTS                                                                           \
    {                                                                                                           \
	printf("Vertex DELTA conflicts\n");					                                \
	for (sInt_32 agent_id = 1; agent_id < m_delta_agent_Conflicts.size(); ++agent_id)                       \
	{                                                                                                       \
	    printf("Agent: %d\n", agent_id);                                                                    \
	    sInt_32 N_steps = m_delta_agent_Conflicts[agent_id].size();	                                        \
	    for (sInt_32 step = 0; step < N_steps; ++step)                                                      \
	    {                                                                                                   \
		printf("  %d: ", step);                                                                         \
		for (VertexIDs_uset::const_iterator conflict = m_delta_agent_Conflicts[agent_id][step].begin(); \
		     conflict != m_delta_agent_Conflicts[agent_id][step].end(); ++conflict)                     \
		{                                                                                               \
		    printf("%d ", *conflict);                                                                   \
		}                                                                                               \
		printf("\n");                                                                                   \
	    }                                                                                                   \
	}                                                                                                       \
    }    


    #define sCBS_SHOW_EDGE_CONFLICTS(agent_edge_Conflicts)		                                                                                                    \
    {                                                                                                                                                                       \
	printf("Edge conflicts\n");                                                                                                                                         \
	for (sInt_32 agent_id = 1; agent_id < agent_edge_Conflicts.size(); ++agent_id)                                                                                      \
	{                                                                                                                                                                   \
	    printf("Agent: %d\n", agent_id);                                                                                                                                \
            for (sInt_32 level = 0; level < agent_edge_Conflicts[agent_id].size(); ++level)                                                                                 \
	    {                                                                                                                                                               \
		printf("%d: ", level);					                                                                                                    \
		for (NeighborIDs_umap::const_iterator neigh = agent_edge_Conflicts[agent_id][level].begin(); neigh != agent_edge_Conflicts[agent_id][level].end(); ++neigh) \
		{							                                                                                                    \
		    printf("%d [", neigh->first);			                                                                                                    \
		    for (VertexIDs_uset::const_iterator vertex = neigh->second.begin(); vertex != neigh->second.end(); ++vertex)                                            \
		    {						                                                                                                            \
			printf("%d ", *vertex);				                                                                                                    \
		    }	                                                                					                                            \
		    printf("] ");					                                                                                                    \
		}							                                                                                                    \
		printf("\n");						                                                                                                    \
	    }                                                                                                                                                               \
	}                                                                                                                                                                   \
    }    

    
    #define sCBS_SHOW_DELTA_EDGE_CONFLICTS	                                                                                                                                            \
    {                                                                                                                                                                                       \
	printf("Edge DELTA conflicts\n");                                                                                                                                                   \
	for (sInt_32 agent_id = 1; agent_id < m_delta_agent_edge_Conflicts.size(); ++agent_id)                                                                                              \
	{                                                                                                                                                                                   \
	    printf("Agent: %d\n", agent_id);                                                                                                                                                \
            for (sInt_32 level = 0; level < m_delta_agent_edge_Conflicts[agent_id].size(); ++level)                                                                                         \
	    {                                                                                                                                                                               \
		printf("%d: ", level);					                                                                                                                    \
		for (NeighborIDs_umap::const_iterator neigh = m_delta_agent_edge_Conflicts[agent_id][level].begin(); neigh != m_delta_agent_edge_Conflicts[agent_id][level].end(); ++neigh) \
		{							                                                                                                                    \
		    printf("%d [", neigh->first);			                                                                                                                    \
		    for (VertexIDs_uset::const_iterator vertex = neigh->second.begin(); vertex != neigh->second.end(); ++vertex)                                                            \
		    {						                                                                                                                            \
			printf("%d ", *vertex);				                                                                                                                    \
		    }	                                                                					   	                                                    \
		    printf("] ");					                                                                                                                    \
		}							                                                                                                                    \
		printf("\n");						                                                                                                                    \
	    }                                                                                                                                                                               \
	}                                                                                                                                                                                   \
    }    


    #define sCBS_SHOW_AGENT_CONFLICTS(Conflicts) 			              \
    {                                                                                 \
        printf("Vertex conflicts\n");                                                 \
	sInt_32 N_steps = Conflicts.size();		                              \
	for (sInt_32 step = 0; step < N_steps; ++step)			              \
	{								              \
	    printf("  %d: ", step);					              \
	    for (VertexIDs_uset::const_iterator conflict = Conflicts[step].begin();   \
		 conflict != Conflicts[step].end(); ++conflict)	                      \
	    {								              \
		printf("%d ", *conflict);				              \
	    }								              \
	    printf("\n");						              \
	}                                                                             \
    }    


    #define sCBS_SHOW_AGENT_EDGE_CONFLICTS(edge_Conflicts)	                                                                         \
    {                                                                                                                                    \
	printf("Edge conflicts\n");                                                                                                      \
        for (sInt_32 level = 0; level < edge_Conflicts.size(); ++level)                                                                  \
	{                                                                                                                                \
	    printf("%d: ", level);                                                                                                       \
	    for (NeighborIDs_umap::const_iterator neigh = edge_Conflicts[level].begin(); neigh != edge_Conflicts[level].end(); ++neigh)  \
	    {                                                                                                                            \
		printf("%d [", neigh->first);                                                                                            \
		for (VertexIDs_uset::const_iterator vertex = neigh->second.begin(); vertex != neigh->second.end(); ++vertex)             \
		{                                                                                                                        \
		    printf("%d ", *vertex);                                                                                              \
		}                                                                                                                        \
		printf("] ");                                                                                                            \
	    }                                                                                                                            \
	    printf("\n");                                                                                                                \
	}                                                                                                                                \
    }


    sInt_32 sCBS::find_NonconflictingSwapping_baseRecompute(const sInstance           &instance,
							    AgentConflicts_vector     &agent_Conflicts,
							    AgentEdgeConflicts_vector &agent_edge_Conflicts,
							    AgentPaths_vector         &agent_Paths,
							    sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!occupations.empty())
	    {
		while (occupations.size() < agent_path_length)
		{
		    occupations.push_back(occupations.back());
		}
	    }
	    else
	    {
		occupations.resize(agent_path_length);		
	    }
	    occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		AgentPaths_vector next_agent_Paths;
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingSwapping_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingSwapping_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Occupation_umap::const_iterator swap_expectation_pred = occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			if (swap_expectation_pred != occupations[i - 1].end()) // swap with occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);
			    
			    if (agent_Paths[swap_expectation_pred->second][ii] != agent_Paths[agent_id][i - 1])
			    {
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = find_NonconflictingSwapping_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
				{
				    agent_Paths = next_agent_Paths;
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = find_NonconflictingSwapping_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
				{
				    agent_Paths = next_agent_Paths;
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				return -1;
			    }
			}
		    }
		    occupations[i][agent_Paths[agent_id][i]] = agent_id;
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

	    for (sInt_32 i = agent_path_length; i < occupations.size(); ++i)
	    {
		AgentPaths_vector next_agent_Paths;
		Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);

		if (occupation_collision != occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingSwapping_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingSwapping_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_pathUpdating(const sInstance           &instance,
							   AgentConflicts_vector     &agent_Conflicts,
							   AgentEdgeConflicts_vector &agent_edge_Conflicts,							   
							   AgentPaths_vector         &agent_Paths,
							   sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }			
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingSwapping(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // swap with occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);
			    
			    if (agent_Paths[swap_expectation_pred->second][ii] != agent_Paths[agent_id][i - 1])
			    {
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = update_NonconflictingSwapping(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				return -1;
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_prioritizedQueue(const sInstance           &instance,
							       AgentConflicts_vector     &agent_Conflicts,
							       AgentEdgeConflicts_vector &agent_edge_Conflicts,
							       AgentPaths_vector         &agent_Paths,
							       sInt_32                    cost_limit) const
    {
	sInt_32 cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Node initial_node(-1);
	sInt_32 initial_cost = 0;

	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.clear();
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 initial_node.m_agent_Conflicts[agent_id],
								 initial_node.m_agent_edge_Conflicts[agent_id],								 
								 initial_node.m_agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }
	    initial_cost += agent_path_length;
	}

	Nodes_mmap search_Queue;
	search_Queue.insert(Nodes_mmap::value_type(initial_cost, initial_node));

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node &best_node = search_Queue.begin()->second;

	    if (best_node.m_upd_agent_id > 0)
	    {       			
		if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								     instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_Paths[best_node.m_upd_agent_id])) < 0)
		{
		    search_Queue.erase(search_Queue.begin());
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingSwapping(instance,
							     best_node.m_agent_Conflicts,
							     best_node.m_agent_edge_Conflicts,							     
							     best_node.m_agent_Paths,
							     cost_limit,
							     search_Queue)) < 0)
	    {
		search_Queue.erase(search_Queue.begin());
		continue;
	    }
	    agent_Paths = best_node.m_agent_Paths;
	    return cummulative;
	}
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_prioritizedTanglement(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,								    
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],					    
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	if ((initial_node.m_cost = analyze_NonconflictingSwapping(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, initial_node.m_tanglement)) > cost_limit)
	{
	    return -1;
	}	
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],						
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingSwapping(instance,
							     best_node.m_agent_Conflicts,
							     best_node.m_agent_edge_Conflicts,							     
							     best_node.m_agent_Paths,
							     cost_limit,
							     search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	}
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_prioritizedCooccupation(const sInstance           &instance,
								      AgentConflicts_vector     &agent_Conflicts,
								      AgentEdgeConflicts_vector &agent_edge_Conflicts,								      
								      AgentPaths_vector         &agent_Paths,
								      sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],					    
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingSwapping(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }	    
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
	        #ifdef sPROFILE
		{				
		    sequencing_begin = clock();
		}
		#endif
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],						
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;
		    
	    if ((cummulative = revise_NonconflictingSwapping(instance,
							     best_node.m_agent_Conflicts,
							     best_node.m_agent_edge_Conflicts,							     
							     best_node.m_agent_Paths,
							     space_Cooccupations,
							     cost_limit,
							     search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_principalCollision(const sInstance           &instance,
								 AgentConflicts_vector     &agent_Conflicts,
								 AgentEdgeConflicts_vector &agent_edge_Conflicts,								 
								 AgentPaths_vector         &agent_Paths,
								 sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingSwapping(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (Nodes_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", node->m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif
				
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;
		    
	    if ((cummulative = examine_NonconflictingSwapping(instance,
							      best_node.m_agent_Conflicts,
							      best_node.m_agent_edge_Conflicts,							      
							      best_node.m_agent_Paths,							      
							      space_Cooccupations,
							      cost_limit,
							      search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_principalCollision_Delta(const sInstance           &instance,
								       AgentConflicts_vector     &agent_Conflicts,
								       AgentEdgeConflicts_vector &agent_edge_Conflicts,
								       AgentPaths_vector         &agent_Paths,
								       sInt_32                    cost_limit)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingSwapping(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingSwappingDelta(instance,
								   best_node.m_node_id,
								   space_Cooccupations,
								   cost_limit,
								   search_Store,
								   search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_principalCollision_DeltaStar(const sInstance           &instance,
									   AgentConflicts_vector     &agent_Conflicts,
									   AgentEdgeConflicts_vector &agent_edge_Conflicts,
									   AgentPaths_vector         &agent_Paths,
									   sInt_32                    cost_limit,
									   sInt_32                    extra_cost)
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findStar_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(agent_id),
						instance.m_goal_configuration.get_AgentLocation(agent_id),
						cost_limit,
						extra_cost,
						initial_node.m_agent_Conflicts[agent_id],
						initial_node.m_agent_edge_Conflicts[agent_id],
						m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingSwapping(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (findStar_NonconflictingSequence(instance.m_environment,
						    instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    cost_limit,
						    extra_cost,
//						    search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						    search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						    m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingSwappingDelta(instance,
								   best_node.m_node_id,
								   space_Cooccupations,
								   cost_limit,
								   search_Store,
								   search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSwapping_principalCollision_DeltaSuperStar(const sInstance           &instance,
										AgentConflicts_vector     &agent_Conflicts,
										AgentEdgeConflicts_vector &agent_edge_Conflicts,
										AgentPaths_vector         &agent_Paths,
										sInt_32                    cost_limit,
										sInt_32                    extra_cost)
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findSuperStar_NonconflictingSequence(instance.m_environment,
						     instance.m_start_configuration.get_AgentLocation(agent_id),
						     instance.m_goal_configuration.get_AgentLocation(agent_id),
						     cost_limit,
						     extra_cost,
						     initial_node.m_agent_Conflicts[agent_id],
						     initial_node.m_agent_edge_Conflicts[agent_id],
						     m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingSwapping(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (findSuperStar_NonconflictingSequence(instance.m_environment,
							 instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 cost_limit,
							 extra_cost,
//						    search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						    search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
							 m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingSwappingDelta(instance,
								   best_node.m_node_id,
								   space_Cooccupations,
								   cost_limit,
								   search_Store,
								   search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }                

    
/*----------------------------------------------------------------------------*/

    sInt_32 sCBS::find_NonconflictingPaths_baseRecompute(const sInstance           &instance,
							 AgentConflicts_vector     &agent_Conflicts,
							 AgentEdgeConflicts_vector &agent_edge_Conflicts,							 
							 AgentPaths_vector         &agent_Paths,
							 sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!occupations.empty())
	    {
		while (occupations.size() < agent_path_length)
		{
		    occupations.push_back(occupations.back());
		}
	    }
	    else
	    {
		occupations.resize(agent_path_length);		
	    }
	    occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		AgentPaths_vector next_agent_Paths;
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingPaths_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingPaths_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);
			    
			    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    if ((cost = find_NonconflictingPaths_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			    {
				agent_Paths = next_agent_Paths;
				return cost;
			    }
			    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    
			    sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    if ((cost = find_NonconflictingPaths_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			    {
				agent_Paths = next_agent_Paths;
				return cost;
			    }
			    sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    return -1;
			}
		    }
		    occupations[i][agent_Paths[agent_id][i]] = agent_id;
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

	    for (sInt_32 i = agent_path_length; i < occupations.size(); ++i)
	    {
		AgentPaths_vector next_agent_Paths;
		Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);

		if (occupation_collision != occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingPaths_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingPaths_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingPaths_pathUpdating(const sInstance           &instance,
							AgentConflicts_vector     &agent_Conflicts,
							AgentEdgeConflicts_vector &agent_edge_Conflicts,							
							AgentPaths_vector         &agent_Paths,
							sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }			
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPaths(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);
			    
			    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			    {
				return cost;
			    }
			    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    
			    sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    if ((cost = update_NonconflictingPaths(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			    {
				return cost;
			    }
			    sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    return -1;
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingPaths_prioritizedQueue(const sInstance           &instance,
							    AgentConflicts_vector     &agent_Conflicts,
							    AgentEdgeConflicts_vector &agent_edge_Conflicts,							    
							    AgentPaths_vector         &agent_Paths,
							    sInt_32                    cost_limit) const
    {
	sInt_32 cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	sInt_32 initial_cost = 0;

	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.clear();
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 initial_node.m_agent_Conflicts[agent_id],
								 initial_node.m_agent_edge_Conflicts[agent_id],
								 initial_node.m_agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }
	    initial_cost += agent_path_length;
	}

	Nodes_mmap search_Queue;
	search_Queue.insert(Nodes_mmap::value_type(initial_cost, initial_node));

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node &best_node = search_Queue.begin()->second;

	    if (best_node.m_upd_agent_id > 0)
	    {
		if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								     instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_Paths[best_node.m_upd_agent_id])) < 0)
		{
		    search_Queue.erase(search_Queue.begin());
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingPaths(instance,
							  best_node.m_agent_Conflicts,
							  best_node.m_agent_edge_Conflicts,
							  best_node.m_agent_Paths,
							  cost_limit,
							  search_Queue)) < 0)
	    {
		search_Queue.erase(search_Queue.begin());
		continue;
	    }
	    agent_Paths = best_node.m_agent_Paths;
	    return cummulative;
	}
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPaths_prioritizedTanglement(const sInstance           &instance,
								 AgentConflicts_vector     &agent_Conflicts,
								 AgentEdgeConflicts_vector &agent_edge_Conflicts,								 
								 AgentPaths_vector         &agent_Paths,
								 sInt_32                    cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],					    
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	if ((initial_node.m_cost = analyze_NonconflictingPaths(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, initial_node.m_tanglement)) > cost_limit)
	{
	    return -1;
	}	
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingPaths(instance,
							  best_node.m_agent_Conflicts,
							  best_node.m_agent_edge_Conflicts,
							  best_node.m_agent_Paths,
							  cost_limit,
							  search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	}
	return -1;
    }

   
    sInt_32 sCBS::find_NonconflictingPaths_prioritizedCooccupation(const sInstance           &instance,
								   AgentConflicts_vector     &agent_Conflicts,
								   AgentEdgeConflicts_vector &agent_edge_Conflicts,								   
								   AgentPaths_vector         &agent_Paths,
								   sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPaths(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f rev:%.3f (bal:%.3f)\n", sequencing_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)(sequencing_cummul + revising_cummul));
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		#ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
                #ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

            #ifdef sPROFILE
	    {		
		revising_begin = clock();
	    }
	    #endif
	    if ((cummulative = revise_NonconflictingPaths(instance,
							  best_node.m_agent_Conflicts,
							  best_node.m_agent_edge_Conflicts,
							  best_node.m_agent_Paths,
							  space_Cooccupations,
							  cost_limit,
							  search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	    #ifdef sPROFILE
	    {		
		revising_end = clock();
		revising_cummul += (revising_end - revising_begin);
	    }
	    #endif
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPaths_principalCollision(const sInstance           &instance,
							      AgentConflicts_vector     &agent_Conflicts,
							      AgentEdgeConflicts_vector &agent_edge_Conflicts,
							      AgentPaths_vector         &agent_Paths,
							      sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);

        #ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPaths(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {
		printf("Times: seq:%.3f rev:%.3f (bal:%.3f)\n", sequencing_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)(sequencing_cummul + revising_cummul));
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{
		    sequencing_begin = clock();
		}
		#endif

		/*
		printf("Queue:%d,%d\n", search_Queue.size());
		printf("update_agent:%d\n", best_node.m_upd_agent_id);
		printf("  conflicts:%d\n", best_node.m_agent_Conflicts[best_node.m_upd_agent_id].size());
		printf("  edge conflicts:%d\n", best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id].size());
		*/
		
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
                #ifdef sPROFILE
		{
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

            #ifdef sPROFILE
	    {
		revising_begin = clock();
	    }
	    #endif
	    if ((cummulative = examine_NonconflictingPaths(instance,
							   best_node.m_agent_Conflicts,
							   best_node.m_agent_edge_Conflicts,
							   best_node.m_agent_Paths,
							   space_Cooccupations,
							   cost_limit,
							   search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
            #ifdef sPROFILE
	    {
		revising_end = clock();
		revising_cummul += (revising_end - revising_begin);
	    }
	    #endif
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPaths_principalCollision_Delta(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit)
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPaths(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingPathsDelta(instance,
								best_node.m_node_id,
								space_Cooccupations,
								cost_limit,
								search_Store,
								search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPaths_principalCollision_DeltaStar(const sInstance           &instance,
									AgentConflicts_vector     &agent_Conflicts,
									AgentEdgeConflicts_vector &agent_edge_Conflicts,
									AgentPaths_vector         &agent_Paths,
									sInt_32                    cost_limit,
									sInt_32                    extra_cost)
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findStar_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(agent_id),
						instance.m_goal_configuration.get_AgentLocation(agent_id),
						cost_limit,
						extra_cost,
						initial_node.m_agent_Conflicts[agent_id],
						initial_node.m_agent_edge_Conflicts[agent_id],
						m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPaths(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif	    

	    /*
	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif
	    */

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		/*
		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		
		*/
		
		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	
		/*
		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif
		*/

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);

		/*
		printf("Node STACK:\n");
		sInt_32 nd_id = best_node.m_node_id;
		std::vector<sInt_32> print_buff;
		while (nd_id > 0)
		{
		    print_buff.push_back(nd_id);
		    nd_id = search_Store[nd_id].m_upper_node_id;
		}
		for (std::vector<sInt_32>::const_reverse_iterator nd = print_buff.rbegin(); nd != print_buff.rend(); ++nd)
		{
		    printf("%d ", *nd);
		}
		printf("\n--------\n");
		*/
	
		if (findStar_NonconflictingSequence(instance.m_environment,
						    instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    cost_limit,
						    extra_cost,
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						    m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingPathsDelta(instance,
								best_node.m_node_id,
								space_Cooccupations,
								cost_limit,
								search_Store,
								search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPaths_principalCollision_DeltaSuperStar(const sInstance           &instance,
									     AgentConflicts_vector     &agent_Conflicts,
									     AgentEdgeConflicts_vector &agent_edge_Conflicts,
									     AgentPaths_vector         &agent_Paths,
									     sInt_32                    cost_limit,
									     sInt_32                    extra_cost)
    {
	sDouble relaxation = 1.0;
	
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findSuperStar_NonconflictingSequence(instance.m_environment,
						     instance.m_start_configuration.get_AgentLocation(agent_id),
						     instance.m_goal_configuration.get_AgentLocation(agent_id),
						     cost_limit,
						     extra_cost,
						     initial_node.m_agent_Conflicts[agent_id],
						     initial_node.m_agent_edge_Conflicts[agent_id],
						     m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPaths(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > relaxation * cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif	    

	    /*
	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif
	    */

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		/*
		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		
		*/
		
		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	
		/*
		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif
		*/

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);

		/*
		printf("Node STACK:\n");
		sInt_32 nd_id = best_node.m_node_id;
		std::vector<sInt_32> print_buff;
		while (nd_id > 0)
		{
		    print_buff.push_back(nd_id);
		    nd_id = search_Store[nd_id].m_upper_node_id;
		}
		for (std::vector<sInt_32>::const_reverse_iterator nd = print_buff.rbegin(); nd != print_buff.rend(); ++nd)
		{
		    printf("%d ", *nd);
		}
		printf("\n--------\n");
		*/
	
		if (findSuperStar_NonconflictingSequence(instance.m_environment,
							 instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 cost_limit,
							 extra_cost,
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
							 m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingPathsDelta(instance,
								best_node.m_node_id,
								space_Cooccupations,
								relaxation * cost_limit,
								search_Store,
								search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }        


/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::find_NonconflictingPermutation_baseRecompute(const sInstance           &instance,
							       AgentConflicts_vector     &agent_Conflicts,
							       AgentEdgeConflicts_vector &agent_edge_Conflicts,							       
							       AgentPaths_vector         &agent_Paths,
							       sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!occupations.empty())
	    {
		while (occupations.size() < agent_path_length)
		{
		    occupations.push_back(occupations.back());
		}
	    }
	    else
	    {
		occupations.resize(agent_path_length);		
	    }
	    occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		AgentPaths_vector next_agent_Paths;
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingPermutation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingPermutation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			if (swap_expectation_pred != occupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);

			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = find_NonconflictingPermutation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
				{
				    agent_Paths = next_agent_Paths;
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = find_NonconflictingPermutation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
				{
				    agent_Paths = next_agent_Paths;
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				return -1;
			    }
			    */
			}
		    }
		    occupations[i][agent_Paths[agent_id][i]] = agent_id;
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

	    for (sInt_32 i = agent_path_length; i < occupations.size(); ++i)
	    {
		AgentPaths_vector next_agent_Paths;
		Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);

		if (occupation_collision != occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingPermutation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingPermutation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_pathUpdating(const sInstance           &instance,
							      AgentConflicts_vector     &agent_Conflicts,
							      AgentEdgeConflicts_vector &agent_edge_Conflicts,							      
							      AgentPaths_vector         &agent_Paths,
							      sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }			
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPermutation(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);

			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {			    
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = update_NonconflictingPermutation(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				return -1;
			    }
			    */
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_prioritizedQueue(const sInstance           &instance,
								  AgentConflicts_vector     &agent_Conflicts,
								  AgentEdgeConflicts_vector &agent_edge_Conflicts,								  
								  AgentPaths_vector         &agent_Paths,
								  sInt_32                    cost_limit) const
    {
	sInt_32 cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	sInt_32 initial_cost = 0;

	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.clear();
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 initial_node.m_agent_Conflicts[agent_id],
								 initial_node.m_agent_edge_Conflicts[agent_id],
								 initial_node.m_agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }
	    initial_cost += agent_path_length;
	}

	Nodes_mmap search_Queue;
	search_Queue.insert(Nodes_mmap::value_type(initial_cost, initial_node));

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node &best_node = search_Queue.begin()->second;

	    if (best_node.m_upd_agent_id > 0)
	    {
		if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								     instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_Paths[best_node.m_upd_agent_id])) < 0)
		{
		    search_Queue.erase(search_Queue.begin());
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingPermutation(instance,
								best_node.m_agent_Conflicts,
								best_node.m_agent_edge_Conflicts,
								best_node.m_agent_Paths,
								cost_limit,
								search_Queue)) < 0)
	    {
		search_Queue.erase(search_Queue.begin());
		continue;
	    }
	    agent_Paths = best_node.m_agent_Paths;
	    return cummulative;
	}
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_prioritizedTanglement(const sInstance           &instance,
								       AgentConflicts_vector     &agent_Conflicts,
								       AgentEdgeConflicts_vector &agent_edge_Conflicts,								       
								       AgentPaths_vector         &agent_Paths,
								       sInt_32                    cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	if ((initial_node.m_cost = analyze_NonconflictingPermutation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, initial_node.m_tanglement)) > cost_limit)
	{
	    return -1;
	}	
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingPermutation(instance,
								best_node.m_agent_Conflicts,
								best_node.m_agent_edge_Conflicts,
								best_node.m_agent_Paths,
								cost_limit,
								search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	}
	return -1;
    }

   
    sInt_32 sCBS::find_NonconflictingPermutation_prioritizedCooccupation(const sInstance           &instance,
									 AgentConflicts_vector     &agent_Conflicts,
									 AgentEdgeConflicts_vector &agent_edge_Conflicts,
									 AgentPaths_vector         &agent_Paths,
									 sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPermutation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f rev:%.3f (bal:%.3f)\n", sequencing_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)(sequencing_cummul + revising_cummul));
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		#ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
                #ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

            #ifdef sPROFILE
	    {		
		revising_begin = clock();
	    }
	    #endif
	    if ((cummulative = revise_NonconflictingPermutation(instance,
								best_node.m_agent_Conflicts,
								best_node.m_agent_edge_Conflicts,
								best_node.m_agent_Paths,
								space_Cooccupations,
								cost_limit,
								search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	    #ifdef sPROFILE
	    {		
		revising_end = clock();
		revising_cummul += (revising_end - revising_begin);
	    }
	    #endif
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_principalCollision(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

        #ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPermutation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {
		printf("Times: seq:%.3f rev:%.3f (bal:%.3f)\n", sequencing_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)(sequencing_cummul + revising_cummul));
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{
		    sequencing_begin = clock();
		}
		#endif
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
                #ifdef sPROFILE
		{
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

            #ifdef sPROFILE
	    {
		revising_begin = clock();
	    }
	    #endif
	    if ((cummulative = examine_NonconflictingPermutation(instance,
								 best_node.m_agent_Conflicts,
								 best_node.m_agent_edge_Conflicts,
								 best_node.m_agent_Paths,
								 space_Cooccupations,
								 cost_limit,
								 search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
            #ifdef sPROFILE
	    {
		revising_end = clock();
		revising_cummul += (revising_end - revising_begin);
	    }
	    #endif
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_principalCollision_Delta(const sInstance           &instance,
									  AgentConflicts_vector     &agent_Conflicts,
									  AgentEdgeConflicts_vector &agent_edge_Conflicts,
									  AgentPaths_vector         &agent_Paths,
									  sInt_32                    cost_limit)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPermutation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingPermutationDelta(instance,
								      best_node.m_node_id,
								      space_Cooccupations,
								      cost_limit,
								      search_Store,
								      search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_principalCollision_DeltaStar(const sInstance           &instance,
									      AgentConflicts_vector     &agent_Conflicts,
									      AgentEdgeConflicts_vector &agent_edge_Conflicts,
									      AgentPaths_vector         &agent_Paths,
									      sInt_32                    cost_limit,
									      sInt_32                    extra_cost)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findStar_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(agent_id),
						instance.m_goal_configuration.get_AgentLocation(agent_id),
						cost_limit,
						extra_cost,
						initial_node.m_agent_Conflicts[agent_id],
						initial_node.m_agent_edge_Conflicts[agent_id],
						m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPermutation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (findStar_NonconflictingSequence(instance.m_environment,
						    instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    cost_limit,
						    extra_cost,
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						    m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingPermutationDelta(instance,
								      best_node.m_node_id,
								      space_Cooccupations,
								      cost_limit,
								      search_Store,
								      search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingPermutation_principalCollision_DeltaSuperStar(const sInstance           &instance,
										   AgentConflicts_vector     &agent_Conflicts,
										   AgentEdgeConflicts_vector &agent_edge_Conflicts,
										   AgentPaths_vector         &agent_Paths,
										   sInt_32                    cost_limit,
										   sInt_32                    extra_cost)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findSuperStar_NonconflictingSequence(instance.m_environment,
						     instance.m_start_configuration.get_AgentLocation(agent_id),
						     instance.m_goal_configuration.get_AgentLocation(agent_id),
						     cost_limit,
						     extra_cost,
						     initial_node.m_agent_Conflicts[agent_id],
						     initial_node.m_agent_edge_Conflicts[agent_id],
						     m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingPermutation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (findSuperStar_NonconflictingSequence(instance.m_environment,
							 instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 cost_limit,
							 extra_cost,
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
							 m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingPermutationDelta(instance,
								      best_node.m_node_id,
								      space_Cooccupations,
								      cost_limit,
								      search_Store,
								      search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }    

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::find_NonconflictingRotation_baseRecompute(const sInstance           &instance,
							    AgentConflicts_vector     &agent_Conflicts,
							    AgentEdgeConflicts_vector &agent_edge_Conflicts,
							    AgentPaths_vector         &agent_Paths,
							    sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!occupations.empty())
	    {
		while (occupations.size() < agent_path_length)
		{
		    occupations.push_back(occupations.back());
		}
	    }
	    else
	    {
		occupations.resize(agent_path_length);		
	    }
	    occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		AgentPaths_vector next_agent_Paths;
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingRotation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = find_NonconflictingRotation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
			{
			    agent_Paths = next_agent_Paths;
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			if (swap_expectation_pred != occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);

			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = find_NonconflictingRotation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
				{
				    agent_Paths = next_agent_Paths;
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = find_NonconflictingRotation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
				{
				    agent_Paths = next_agent_Paths;
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				return -1;
			    }
			}
		    }
		    occupations[i][agent_Paths[agent_id][i]] = agent_id;
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

	    for (sInt_32 i = agent_path_length; i < occupations.size(); ++i)
	    {
		AgentPaths_vector next_agent_Paths;
		Occupation_umap::const_iterator occupation_collision = occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);

		if (occupation_collision != occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingRotation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = find_NonconflictingRotation_baseRecompute(instance, agent_Conflicts, agent_edge_Conflicts, next_agent_Paths, cost_limit)) >= 0)
		    {
			agent_Paths = next_agent_Paths;
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingRotation_pathUpdating(const sInstance           &instance,
							   AgentConflicts_vector     &agent_Conflicts,
							   AgentEdgeConflicts_vector &agent_edge_Conflicts,
							   AgentPaths_vector         &agent_Paths,
							   sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	agent_Paths.clear();
	agent_Paths.resize(N_agents + 1);
	
	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 agent_Conflicts[agent_id],
								 agent_edge_Conflicts[agent_id],
								 agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }			
	}

	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();

		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingRotation(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			return -1;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);

			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {			    
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = update_NonconflictingRotation(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				return -1;
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
			
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::find_NonconflictingRotation_prioritizedQueue(const sInstance           &instance,
							       AgentConflicts_vector     &agent_Conflicts,
							       AgentEdgeConflicts_vector &agent_edge_Conflicts,
							       AgentPaths_vector         &agent_Paths,
							       sInt_32                    cost_limit) const
    {
	sInt_32 cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	sInt_32 initial_cost = 0;

	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.clear();
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(agent_id),
								 instance.m_goal_configuration.get_AgentLocation(agent_id),
								 initial_node.m_agent_Conflicts[agent_id],
								 initial_node.m_agent_edge_Conflicts[agent_id],
								 initial_node.m_agent_Paths[agent_id])) < 0)
	    {
		return -1;
	    }
	    initial_cost += agent_path_length;
	}

	Nodes_mmap search_Queue;
	search_Queue.insert(Nodes_mmap::value_type(initial_cost, initial_node));

	while (!search_Queue.empty())	    
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node &best_node = search_Queue.begin()->second;

	    if (best_node.m_upd_agent_id > 0)
	    {
		if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								     instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
								     best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
								     best_node.m_agent_Paths[best_node.m_upd_agent_id])) < 0)
		{
		    search_Queue.erase(search_Queue.begin());
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingRotation(instance,
							     best_node.m_agent_Conflicts,
							     best_node.m_agent_edge_Conflicts,
							     best_node.m_agent_Paths,
							     cost_limit,
							     search_Queue)) < 0)
	    {
		search_Queue.erase(search_Queue.begin());
		continue;
	    }
	    agent_Paths = best_node.m_agent_Paths;
	    return cummulative;
	}
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingRotation_prioritizedTanglement(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit) const
    {
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	if ((initial_node.m_cost = analyze_NonconflictingRotation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, initial_node.m_tanglement)) > cost_limit)
	{
	    return -1;
	}	
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
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

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    
	    
	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
	    }

	    if ((cummulative = revise_NonconflictingRotation(instance,
							     best_node.m_agent_Conflicts,
							     best_node.m_agent_edge_Conflicts,
							     best_node.m_agent_Paths,
							     cost_limit,
							     search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	}
	return -1;
    }

   
    sInt_32 sCBS::find_NonconflictingRotation_prioritizedCooccupation(const sInstance           &instance,
								      AgentConflicts_vector     &agent_Conflicts,
								      AgentEdgeConflicts_vector &agent_edge_Conflicts,
								      AgentPaths_vector         &agent_Paths,
								      sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingRotation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f rev:%.3f (bal:%.3f)\n", sequencing_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)(sequencing_cummul + revising_cummul));
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
		#ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
                #ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

            #ifdef sPROFILE
	    {		
		revising_begin = clock();
	    }
            #endif
	    if ((cummulative = revise_NonconflictingRotation(instance,
							     best_node.m_agent_Conflicts,
							     best_node.m_agent_edge_Conflicts,
							     best_node.m_agent_Paths,
							     space_Cooccupations,
							     cost_limit,
							     search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
	    #ifdef sPROFILE
	    {		
		revising_end = clock();
		revising_cummul += (revising_end - revising_begin);
	    }
	    #endif
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingRotation_principalCollision(const sInstance           &instance,
								 AgentConflicts_vector     &agent_Conflicts,
								 AgentEdgeConflicts_vector &agent_edge_Conflicts,
								 AgentPaths_vector         &agent_Paths,
								 sInt_32                    cost_limit) const
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;
	initial_node.m_agent_Paths.resize(N_agents + 1);

        #ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    initial_node.m_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }
	}
	
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingRotation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, initial_node.m_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	Nodes_mset search_Queue;
	search_Queue.insert(initial_node);	

	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {
		printf("Times: seq:%.3f rev:%.3f (bal:%.3f)\n", sequencing_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)CLOCKS_PER_SEC, revising_cummul / (double)(sequencing_cummul + revising_cummul));
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (Nodes_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", node->m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    Node best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (best_node.m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{
		    sequencing_begin = clock();
		}
		#endif
		
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(best_node.m_upd_agent_id),
						best_node.m_agent_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_edge_Conflicts[best_node.m_upd_agent_id],
						best_node.m_agent_Paths[best_node.m_upd_agent_id]) < 0)
		{
		    continue;
		}
                #ifdef sPROFILE
		{
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

            #ifdef sPROFILE
	    {
		revising_begin = clock();
	    }
	    #endif
	    if ((cummulative = examine_NonconflictingRotation(instance,
							      best_node.m_agent_Conflicts,
							      best_node.m_agent_edge_Conflicts,
							      best_node.m_agent_Paths,
							      space_Cooccupations,
							      cost_limit,
							      search_Queue)) >= 0)
	    {
		agent_Paths = best_node.m_agent_Paths;
		return cummulative;
	    }
            #ifdef sPROFILE
	    {
		revising_end = clock();
		revising_cummul += (revising_end - revising_begin);
	    }
	    #endif
	}	
	return -1;
    }        


    sInt_32 sCBS::find_NonconflictingRotation_principalCollision_Delta(const sInstance           &instance,
								       AgentConflicts_vector     &agent_Conflicts,
								       AgentEdgeConflicts_vector &agent_edge_Conflicts,
								       AgentPaths_vector         &agent_Paths,
								       sInt_32                    cost_limit)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (find_NonconflictingSequence(instance.m_environment,
					    instance.m_start_configuration.get_AgentLocation(agent_id),
					    instance.m_goal_configuration.get_AgentLocation(agent_id),
					    initial_node.m_agent_Conflicts[agent_id],
					    initial_node.m_agent_edge_Conflicts[agent_id],
					    m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingRotation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (find_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
//						search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//						search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingRotationDelta(instance,
								   best_node.m_node_id,
								   space_Cooccupations,
								   cost_limit,
								   search_Store,
								   search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingRotation_principalCollision_DeltaStar(const sInstance           &instance,
									   AgentConflicts_vector     &agent_Conflicts,
									   AgentEdgeConflicts_vector &agent_edge_Conflicts,
									   AgentPaths_vector         &agent_Paths,
									   sInt_32                    cost_limit,
									   sInt_32                    extra_cost)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findStar_NonconflictingSequence(instance.m_environment,
						instance.m_start_configuration.get_AgentLocation(agent_id),
						instance.m_goal_configuration.get_AgentLocation(agent_id),
						cost_limit,
						extra_cost,
						initial_node.m_agent_Conflicts[agent_id],
						initial_node.m_agent_edge_Conflicts[agent_id],
						m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingRotation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (findStar_NonconflictingSequence(instance.m_environment,
						    instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
						    cost_limit,
						    extra_cost,
//						    search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//				  		    search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
						    m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
						    m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingRotationDelta(instance,
								   best_node.m_node_id,
								   space_Cooccupations,
								   cost_limit,
								   search_Store,
								   search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingRotation_principalCollision_DeltaSuperStar(const sInstance           &instance,
										AgentConflicts_vector     &agent_Conflicts,
										AgentEdgeConflicts_vector &agent_edge_Conflicts,
										AgentPaths_vector         &agent_Paths,
										sInt_32                    cost_limit,
										sInt_32                    extra_cost)
    {    
	sInt_32 cummulative;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();	
	
	Node initial_node(-1);

	initial_node.m_node_id = 0;
	initial_node.m_upper_node_id = -1;

	initial_node.m_next_conflict = NULL;
	initial_node.m_next_edge_conflict = NULL;

	initial_node.m_next_path = NULL;
	initial_node.m_prev_path = NULL;
	
	initial_node.m_agent_Conflicts = agent_Conflicts;
	initial_node.m_agent_edge_Conflicts = agent_edge_Conflicts;	
	initial_node.m_agent_Paths.resize(N_agents + 1);

	#ifdef sPROFILE
	{		
	    sequencing_cummul = revising_cummul = analyzing_cummul = collecting_cummul = 0;
	}
	#endif

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    if (findSuperStar_NonconflictingSequence(instance.m_environment,
						     instance.m_start_configuration.get_AgentLocation(agent_id),
						     instance.m_goal_configuration.get_AgentLocation(agent_id),
						     cost_limit,
						     extra_cost,
						     initial_node.m_agent_Conflicts[agent_id],
						     initial_node.m_agent_edge_Conflicts[agent_id],
						     m_first_agent_Paths[agent_id]) < 0)
	    {
		return -1;
	    }	    
	    m_delta_agent_Paths[agent_id] = m_first_agent_Paths[agent_id];
	    m_delta_path_node_IDs[agent_id] = initial_node.m_node_id;
	}
	{
	    Cooccupations_vector space_Cooccupations;
	    
	    if ((initial_node.m_cost = analyze_NonconflictingRotation(instance, initial_node.m_agent_Conflicts, initial_node.m_agent_edge_Conflicts, m_first_agent_Paths, space_Cooccupations, initial_node.m_tanglement)) > cost_limit)
	    {
		return -1;
	    }
	}
	
	Nodes_vector search_Store;
	NodeReferences_mset search_Queue;

	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
	
	while (!search_Queue.empty())
	{
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	    
	    #ifdef sPROFILE
	    {		
		printf("Times: seq:%.3f\n", sequencing_cummul / (double)CLOCKS_PER_SEC);
	    }
	    #endif
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    #ifdef sDEBUG
	    {
		printf("Search queue size:%ld\n", search_Queue.size());

		printf("Search queue agents: ");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d ", search_Store[node->m_node_id].m_upd_agent_id);
		}
		printf("\n");
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

	    if (search_Store[best_node.m_node_id].m_upd_agent_id > 0)
	    {
                #ifdef sPROFILE
		{		
		    sequencing_begin = clock();
		}
		#endif

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif		

		rebuild_NodeConflictsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
					   best_node.m_node_id,
					   search_Store);	

		#ifdef sDEBUG
		{
		    sCBS_SHOW_AGENT_CONFLICTS(m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		    sCBS_SHOW_AGENT_EDGE_CONFLICTS(m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id]);
		}
		#endif

		rebuild_NodePathsDelta(search_Store[best_node.m_node_id].m_upd_agent_id,
				       best_node.m_node_id,
				       search_Store);		
		search_Store[best_node.m_node_id].m_prev_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
	
		if (findSuperStar_NonconflictingSequence(instance.m_environment,
							 instance.m_start_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 instance.m_goal_configuration.get_AgentLocation(search_Store[best_node.m_node_id].m_upd_agent_id),
							 cost_limit,
							 extra_cost,
//						    search_Store[best_node.m_node_id].m_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
//				  		    search_Store[best_node.m_node_id].m_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],						
							 m_delta_agent_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_edge_Conflicts[search_Store[best_node.m_node_id].m_upd_agent_id],
							 m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]) < 0)
		{
		    continue;
		}
		search_Store[best_node.m_node_id].m_next_path = new VertexIDs_vector(m_delta_agent_Paths[search_Store[best_node.m_node_id].m_upd_agent_id]);
		m_delta_path_node_IDs[search_Store[best_node.m_node_id].m_upd_agent_id] = best_node.m_node_id;

		#ifdef sPROFILE
		{		
		    sequencing_end = clock();
		    sequencing_cummul += (sequencing_end - sequencing_begin);
		}
		#endif
	    }
	    Cooccupations_vector space_Cooccupations;

	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		rebuild_NodePathsDelta(agent_id,
				       best_node.m_node_id,
				       search_Store);
	    }	    
	    if ((cummulative = examine_NonconflictingRotationDelta(instance,
								   best_node.m_node_id,
								   space_Cooccupations,
								   cost_limit,
								   search_Store,
								   search_Queue)) >= 0)
	    {
		agent_Paths = m_delta_agent_Paths;
		return cummulative;
	    }
	}	
	return -1;
    }    
    
    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::revise_NonconflictingSwapping(const sInstance           &instance,
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						const AgentPaths_vector   &agent_Paths,
						sInt_32                    cost_limit,
						Nodes_mmap                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		       
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
			
			next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // swap with occupied
			{
			    if (i < agent_Paths[swap_expectation_pred->second].size())
			    {
				if (agent_Paths[swap_expectation_pred->second][i] != agent_Paths[agent_id][i - 1])
				{
				    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, agent_id, i-1, agent_Paths[agent_id][i-1], agent_Paths[agent_id][i]);
				    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
				    
				    next_node = Node(swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, swap_expectation_pred->second, i, agent_Paths[swap_expectation_pred->second][i-1], agent_Paths[swap_expectation_pred->second][i]);
				    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
				    
				    return -1;
				}
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
		    
		    next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}

	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingSwapping(const sInstance           &instance,
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						const AgentPaths_vector   &agent_Paths,
						sInt_32                    cost_limit,
						Nodes_mset                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}

	if ((cummulative = analyze_NonconflictingSwapping(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, tanglement)) > cost_limit)
	{
	    return -1;
	}	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);
			
			next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // swap with occupied
			{
			    if (i < agent_Paths[swap_expectation_pred->second].size())
			    {
				if (agent_Paths[swap_expectation_pred->second][i] != agent_Paths[agent_id][i - 1])
				{
				    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, agent_id, i-1, agent_Paths[agent_id][i-1], agent_Paths[agent_id][i]);
				    search_Queue.insert(next_node);
				
				    next_node = Node(cummulative, tanglement, swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, swap_expectation_pred->second, i-1, agent_Paths[swap_expectation_pred->second][i-1], agent_Paths[swap_expectation_pred->second][i]);
				    search_Queue.insert(next_node);

				    return -1;
				}
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingSwapping(const sInstance           &instance,
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						const AgentPaths_vector   &agent_Paths,
						Cooccupations_vector      &space_Cooccupations,
						sInt_32                    cost_limit,
						Nodes_mset                &search_Queue) const
    {
	Collisions_mset agent_Collisions;
	EdgeCollisions_mset agent_edge_Collisions;	
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	if ((cummulative = analyze_NonconflictingSwapping(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    return -1;
	}
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
				agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				if (i < agent_Paths[*exp_agent].size())
				{
				    if (agent_Paths[*exp_agent][i] != agent_Paths[agent_id][i - 1])
				    {
					if (*exp_agent != agent_id)
					{
					    agent_edge_Collisions.insert(EdgeCollision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
										       i-1,
										       agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
										       agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]));
					}
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
			    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]));
			}
		    }		    
		}
	    }
	}

	if (!agent_Collisions.empty())
	{
	    Collisions_mset::const_iterator principal_collision = agent_Collisions.begin();
	    if (!agent_edge_Collisions.empty())
	    {
		EdgeCollisions_mset::const_iterator principal_edge_collision = agent_edge_Collisions.begin();

		if (*principal_edge_collision < *principal_collision)
		{
		    Node next_node(cummulative, tanglement, principal_edge_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_A_id, principal_edge_collision->m_level_A, principal_edge_collision->m_edge_A_u_id, principal_edge_collision->m_edge_A_v_id);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, principal_edge_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_B_id, principal_edge_collision->m_level_B, principal_edge_collision->m_edge_B_u_id, principal_edge_collision->m_edge_B_v_id);	    
		    search_Queue.insert(next_node);	    		    
		    return -1;		    
		}
		else
		{
		    Node next_node(cummulative, tanglement, principal_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_A_id, principal_collision->m_level_A, principal_collision->m_vertex_A_id);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, principal_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_B_id, principal_collision->m_level_B, principal_collision->m_vertex_B_id);
		    search_Queue.insert(next_node);		    
		    return -1;
		}
	    }
	    else
	    {
		Node next_node(cummulative, tanglement, principal_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_A_id, principal_collision->m_level_A, principal_collision->m_vertex_A_id);
		search_Queue.insert(next_node);
		
		next_node = Node(cummulative, tanglement, principal_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_B_id, principal_collision->m_level_B, principal_collision->m_vertex_B_id);
		search_Queue.insert(next_node);		    
		return -1;		
	    }
	}
	else
	{
	    if (!agent_edge_Collisions.empty())
	    {
		EdgeCollisions_mset::const_iterator principal_edge_collision = agent_edge_Collisions.begin();

		Node next_node(cummulative, tanglement, principal_edge_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_A_id, principal_edge_collision->m_level_A, principal_edge_collision->m_edge_A_u_id, principal_edge_collision->m_edge_A_v_id);
		search_Queue.insert(next_node);
	    
		next_node = Node(cummulative, tanglement, principal_edge_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_B_id, principal_edge_collision->m_level_B, principal_edge_collision->m_edge_B_u_id, principal_edge_collision->m_edge_B_v_id);	    
		search_Queue.insert(next_node);	    		
		return -1;
	    }
	}
	return cummulative;
    }

    
    #define sCBS_INTRODUCE_SWAPPING_VERTEX_CONFLICTS                                                                                                   \
    {                                                                                                                                                  \
        Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);               \
        sCBS_ADD_NODE_AGENT_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
        search_Queue.insert(next_node_1);					                                                                       \
        Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);               \
        sCBS_ADD_NODE_AGENT_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
        search_Queue.insert(next_node_2);					                                                                       \
    }

    
    #define sCBS_INTRODUCE_SWAPPING_EDGE_CONFLICTS                                                                                                                                                                 \
    {                                                                                                                                                                                                              \
        Node next_node_1(cummulative, tanglement, principal_edge_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);                                                                      \
        sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id); \
        search_Queue.insert(next_node_1);	                                                                                                                                                                   \
        Node next_node_2(cummulative, tanglement, principal_edge_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);                                                                      \
        sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id); \
        search_Queue.insert(next_node_2);                                                                                                                                                                          \
    }
    
    
    sInt_32 sCBS::examine_NonconflictingSwapping(const sInstance           &instance,
						 AgentConflicts_vector     &agent_Conflicts,
						 AgentEdgeConflicts_vector &agent_edge_Conflicts,
						 const AgentPaths_vector   &agent_Paths,
						 Cooccupations_vector      &space_Cooccupations,
						 sInt_32                    cost_limit,
						 Nodes_mset                &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);
	EdgeCollision principal_edge_collision(sINT_32_MAX, 1, 1, 0, 0, 0, 0, 0);

        #ifdef sDEBUG
	{
	    printf("----> Begin <----\n");
	    
	    sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		printf("Agnt %d: ", agent_id);
		
		for (sInt_32 i = 0; i < agent_Paths[agent_id].size(); ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }

	    sCBS_SHOW_CONFLICTS(agent_Conflicts);
	    sCBS_SHOW_EDGE_CONFLICTS(agent_edge_Conflicts);

	    printf("----> End <----\n");
	}
        #endif	
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}				
	if ((cummulative = analyze_NonconflictingSwapping(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
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
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				if (i < agent_Paths[*exp_agent].size())
				{
				    if (agent_Paths[*exp_agent][i] != agent_Paths[agent_id][i-1])
				    {
					if (*exp_agent != agent_id)
					{
					    EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									      i-1,
									      agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									      agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					    
					    if (next_edge_collision < principal_edge_collision)
					    {
						principal_edge_collision = next_edge_collision;
					    }
					}
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
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}

	sInt_32 cummulative_result = cummulative;

	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		if (principal_edge_collision < principal_collision)
		{
		    sCBS_INTRODUCE_SWAPPING_EDGE_CONFLICTS;
		    cummulative_result = -1;
		}
		else
		{
		    sCBS_INTRODUCE_SWAPPING_VERTEX_CONFLICTS;
		    cummulative_result = -1;		    
		}
	    }
	    else
	    {
		sCBS_INTRODUCE_SWAPPING_VERTEX_CONFLICTS;
		cummulative_result = -1;
	    }
	}
	else
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		sCBS_INTRODUCE_SWAPPING_EDGE_CONFLICTS;
		cummulative_result = -1;
	    }
	}
	    
	return cummulative_result;
    }


    #define sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(node, agent_id, step, vertex_id)  \
    {                                                                            \
        (node).m_next_conflict = new Conflict(vertex_id, agent_id, step);        \
        (node).m_next_edge_conflict = NULL;                                      \
	(node).m_next_path = NULL;                                               \
        (node).m_prev_path = NULL;                                               \
    }

/*
	if ((node).m_agent_Conflicts[(agent_id)].size() <= (step))               \
	{                                                                        \
	    (node).m_agent_Conflicts[(agent_id)].resize((step) + 1);             \
	}                                                                        \
	(node).m_agent_Conflicts[(agent_id)][(step)].insert((vertex_id));        \
*/  


    #define sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(node, agent_id, step, edge_u_id, edge_v_id) \
    {                                                                                           \
        (node).m_next_edge_conflict = new EdgeConflict(edge_u_id, edge_v_id, agent_id, step);   \
        (node).m_next_conflict = NULL;	                                                        \
	(node).m_next_path = NULL;                                                              \
        (node).m_prev_path = NULL;                                                              \
    }

    /*
	if ((node).m_agent_edge_Conflicts[(agent_id)].size() <= (step))                         \
	{                                                                                       \
	    (node).m_agent_edge_Conflicts[(agent_id)].resize((step) + 1);                       \
	}                                                                                       \
	(node).m_agent_edge_Conflicts[(agent_id)][(step)][(edge_u_id)].insert((edge_v_id));     \
    */


    #define sCBS_INTRODUCE_SWAPPING_VERTEX_DELTA_CONFLICTS(upper_node_id)                                              	                                     \
    {								 	                                                                                     \
        Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id);                                                                         \
        next_node_1.m_node_id = search_Store.size();                                                                                                         \
        next_node_1.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
	sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_1.m_node_id;                                                                                \
        search_Store.push_back(next_node_1);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_1.m_node_id, &search_Store));	                                                                     \
        Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id);                                                                         \
        next_node_2.m_node_id = search_Store.size();		    	                                                                                     \
        next_node_2.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
        sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
        search_Store[(upper_node_id)].m_right_node_id = next_node_2.m_node_id;                                                                               \
        search_Store.push_back(next_node_2);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_2.m_node_id, &search_Store));	                                                                     \
    }

    /*
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
    */
    
    #define sCBS_INTRODUCE_SWAPPING_EDGE_DELTA_CONFLICTS(upper_node_id)                                                                                                                                                  \
    {                                                                                                                                                                                                                    \
        Node next_node_1(cummulative, tanglement, principal_edge_collision.m_agent_A_id);                                                                                                                                \
        next_node_1.m_node_id = search_Store.size();                                                                                                                                                                     \
        next_node_1.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                                                                                           \
        sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_1.m_node_id;                                                                                                                                            \
        search_Store.push_back(next_node_1);				                                                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_1.m_node_id, &search_Store));	                                                                                                                                 \
        Node next_node_2(cummulative, tanglement, principal_edge_collision.m_agent_B_id);                                                                                                                                \
        next_node_2.m_node_id = search_Store.size();                                                                                                                                                                     \
        next_node_2.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                                                                                           \
        sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_2.m_node_id;                                                                                                                                            \
        search_Store.push_back(next_node_2);                                                                                                                                                                             \
        search_Queue.insert(NodeReference(next_node_2.m_node_id, &search_Store));	                                                                                                                                 \
    }

    /*
            sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id); \
        sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id); \    
    */

    sInt_32 sCBS::examine_NonconflictingSwappingDelta(const sInstance      &instance,
						      sInt_32               upper_node_id,
						      Cooccupations_vector &space_Cooccupations,
						      sInt_32               cost_limit,
						      Nodes_vector         &search_Store,
						      NodeReferences_mset  &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);
	EdgeCollision principal_edge_collision(sINT_32_MAX, 1, 1, 0, 0, 0, 0, 0);

	const AgentPaths_vector &agent_Paths = m_delta_agent_Paths;

        #ifdef sDEBUG
	{
	    printf("----> Begin <----\n");
	    	    
	    for (sInt_32 agent_id = 1; agent_id < m_delta_agent_Paths.size(); ++agent_id)
	    {
		printf("Agnt %d: ", agent_id);
		
		for (sInt_32 i = 0; i < m_delta_agent_Paths[agent_id].size(); ++i)
		{
		    printf("%d ", m_delta_agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	    
	    printf("----> End <----\n");
	}
        #endif

	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}
	if ((cummulative = analyze_NonconflictingSwapping(instance, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
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
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
			    }
			}
		    }
		    
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				if (i < agent_Paths[*exp_agent].size())
				{
				    if (agent_Paths[*exp_agent][i] != agent_Paths[agent_id][i-1])
				    {
					if (*exp_agent != agent_id)
					{
					    EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									      i-1,
									      agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									      agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					    if (next_edge_collision < principal_edge_collision)
					    {
						principal_edge_collision = next_edge_collision;
					    }
					}
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
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}

	sInt_32 cummulative_result = cummulative;

	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		if (principal_edge_collision < principal_collision)
		{
		    sCBS_INTRODUCE_SWAPPING_EDGE_DELTA_CONFLICTS(upper_node_id);
		    cummulative_result = -1;
		}
		else
		{
		    sCBS_INTRODUCE_SWAPPING_VERTEX_DELTA_CONFLICTS(upper_node_id);
		    cummulative_result = -1;
		}
	    }
	    else
	    {
		sCBS_INTRODUCE_SWAPPING_VERTEX_DELTA_CONFLICTS(upper_node_id);
		cummulative_result = -1;
	    }
	}
	else
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		sCBS_INTRODUCE_SWAPPING_EDGE_DELTA_CONFLICTS(upper_node_id);
		cummulative_result = -1;
	    }
	}
	return cummulative_result;
    }                


/*----------------------------------------------------------------------------*/

    sInt_32 sCBS::analyze_NonconflictingSwapping(const sInstance         &instance,
						 const AgentPaths_vector &agent_Paths,
						 sInt_32                 &tanglement) const
    {
	AgentConflicts_vector dummy_Conflicts;
	AgentEdgeConflicts_vector dummy_edge_Conflicts;
	
	return analyze_NonconflictingSwapping(instance,
					      dummy_Conflicts,
					      dummy_edge_Conflicts,
					      agent_Paths,
					      tanglement);
    }
    
    sInt_32 sCBS::analyze_NonconflictingSwapping(const sInstance           &instance,
						 AgentConflicts_vector     &sUNUSED(agent_Conflicts),
						 AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
						 const AgentPaths_vector   &agent_Paths,
						 sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;

	sInt_32 cummulative = 0;
	tanglement = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			++tanglement;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // swap with occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);
			    
			    if (agent_Paths[swap_expectation_pred->second][ii] != agent_Paths[agent_id][i - 1])
			    {
				++tanglement;
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    ++tanglement;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::analyze_NonconflictingSwapping(const sInstance         &instance,
						 const AgentPaths_vector &agent_Paths,
						 Cooccupations_vector    &space_Cooccupations,
						 sInt_32                 &tanglement) const
    {
	AgentConflicts_vector dummy_Conflicts;
	AgentEdgeConflicts_vector dummy_edge_Conflicts;

	return analyze_NonconflictingSwapping(instance,
					      dummy_Conflicts,
					      dummy_edge_Conflicts,
					      agent_Paths,
					      space_Cooccupations,
					      tanglement);

    }

    
    sInt_32 sCBS::analyze_NonconflictingSwapping(const sInstance           &instance,
						 AgentConflicts_vector     &sUNUSED(agent_Conflicts),
						 AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
						 const AgentPaths_vector   &agent_Paths,
						 Cooccupations_vector      &space_Cooccupations,
						 sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	tanglement = 0;

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
				++tanglement;
			    }
			}
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // swap with occupied
			{
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				if (*exp_agent != agent_id)
				{				
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
			    
				    if (agent_Paths[*exp_agent][ii] != agent_Paths[agent_id][i - 1])
				    {
					++tanglement;
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
			    ++tanglement;
			}
		    }
		}
	    }
	}
	return cummulative;
    }    

  

    sInt_32 sCBS::update_NonconflictingSwapping(sInt_32                    upd_agent_id,
						const sInstance           &instance,
						Occupations_vector        &sUNUSED(space_Occupations_),
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,						
						AgentPaths_vector         &agent_Paths,
						sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	VertexIDs_vector stored_path = agent_Paths[upd_agent_id];
	agent_Paths[upd_agent_id].clear();

	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(upd_agent_id),
								 instance.m_goal_configuration.get_AgentLocation(upd_agent_id),
								 agent_Conflicts[upd_agent_id],
								 agent_edge_Conflicts[upd_agent_id],
								 agent_Paths[upd_agent_id])) < 0)
	    {
		agent_Paths[upd_agent_id] = stored_path;
		return -1;
	    }
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);
	    }
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}
	if (cummulative > cost_limit)
	{
	    agent_Paths[upd_agent_id] = stored_path;
	    return -1;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
	    
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
	
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingSwapping(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			agent_Paths[upd_agent_id] = stored_path;
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper swap
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // swap with occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);
			    
			    if (agent_Paths[swap_expectation_pred->second][ii] != agent_Paths[agent_id][i - 1])
			    {
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = update_NonconflictingSwapping(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				agent_Paths[upd_agent_id] = stored_path;
				return -1;
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
		    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingSwapping(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingSwapping(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    agent_Paths[upd_agent_id] = stored_path;
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }	    
	}
	return cummulative;
    }        


/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::revise_NonconflictingPaths(const sInstance           &instance,
					     AgentConflicts_vector     &agent_Conflicts,
					     AgentEdgeConflicts_vector &agent_edge_Conflicts,
					     const AgentPaths_vector   &agent_Paths,
					     sInt_32                    cost_limit,
					     Nodes_mmap                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		       
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
			
			next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);
			    
			    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
				
			    next_node = Node(swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

			    return -1;
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
		    
		    next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}

	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingPaths(const sInstance           &instance,
					     AgentConflicts_vector     &agent_Conflicts,
					     AgentEdgeConflicts_vector &agent_edge_Conflicts,
					     const AgentPaths_vector   &agent_Paths,
					     sInt_32                    cost_limit,
					     Nodes_mset                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}

	if ((cummulative = analyze_NonconflictingPaths(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, tanglement)) > cost_limit)
	{
	    return -1;
	}	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);
			
			next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);
			    
			    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			    search_Queue.insert(next_node);
				
			    next_node = Node(cummulative, tanglement, swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    search_Queue.insert(next_node);

			    return -1;
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingPaths(const sInstance           &instance,
					     AgentConflicts_vector     &agent_Conflicts,
					     AgentEdgeConflicts_vector &agent_edge_Conflicts,
					     const AgentPaths_vector   &agent_Paths,
					     Cooccupations_vector      &space_Cooccupations,
					     sInt_32                    cost_limit,
					     Nodes_mset                &search_Queue) const
    {
	Collisions_mset agent_Collisions;
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	if ((cummulative = analyze_NonconflictingPaths(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    return -1;
	}
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
/*
		    if (!agent_Collisions.empty())
		    {
			Collisions_mset::const_iterator leading_collision = agent_Collisions.begin();

			if (occupation_collision != space_Cooccupations[i].end() && leading_collision->m_cooccupation > occupation_collision->second.size())
			{
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				if (*collide_agent > agent_id)
				{
				    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
				}
			    }
			}
		    }
		    else
*/
		    {
			if (occupation_collision != space_Cooccupations[i].end())
			{
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				if (*collide_agent > agent_id)
				{
				    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
				}
			    }
			}
		    }
/*
		    if (!agent_Collisions.empty())
		    {
			Collisions_mset::const_iterator leading_collision = agent_Collisions.begin();
			
			if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
			{
			    Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			    
			    if (swap_expectation_pred != space_Cooccupations[i - 1].end() && leading_collision->m_cooccupation > swap_expectation_pred->second.size()) // move into occupied
			    {
				for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
				{
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				    
				    if (*exp_agent != agent_id)
				    {
					agent_Collisions.insert(Collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]));
				    }
				}
			    }
			}			
		    }
		    else
*/
		    {
			if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
			{
			    Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			    
			    if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied
			    {
				for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
				{
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i - 1);
				    
				    if (*exp_agent != agent_id)
				    {
					agent_Collisions.insert(Collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]));
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
			    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]));
			}
		    }		    
		}
	    }
	}
	
	if (!agent_Collisions.empty())
	{
	    Collisions_mset::const_iterator principal_collision = agent_Collisions.begin();

	    Node next_node(cummulative, tanglement, principal_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_A_id, principal_collision->m_level_A, principal_collision->m_vertex_A_id);
	    search_Queue.insert(next_node);

	    next_node = Node(cummulative, tanglement, principal_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_B_id, principal_collision->m_level_B, principal_collision->m_vertex_B_id);
	    search_Queue.insert(next_node);

	    return -1;
	}       			
	return cummulative;
    }

    
    sInt_32 sCBS::examine_NonconflictingPaths(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      const AgentPaths_vector   &agent_Paths,
					      Cooccupations_vector      &space_Cooccupations,
					      sInt_32                    cost_limit,
					      Nodes_mset                &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);	
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		
	if ((cummulative = analyze_NonconflictingPaths(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
	#ifdef sPROFILE
	{	
	    analyzing_end = clock();
	    analyzing_cummul += (analyzing_end - analyzing_begin);
	    
	    collecting_begin = clock();
	}
	#endif		    
	
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
				
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
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
				
				if (*exp_agent != agent_id)
				{
				    Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
				    if (next_collision < principal_collision)
				    {
					principal_collision = next_collision;
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
			
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}
	#ifdef sPROFILE
	{		
	    collecting_end = clock();
	    collecting_cummul += (collecting_end - collecting_begin);
	}
	#endif
	
	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    Node next_node(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id);
	    search_Queue.insert(next_node);

	    next_node = Node(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    search_Queue.insert(next_node);

	    return -1;
	}
        #ifdef sPROFILE
	{		
	    printf("** Times: ana:%.3f col:%.3f (bal:%.3f)\n", analyzing_cummul / (double)CLOCKS_PER_SEC, collecting_cummul / (double)CLOCKS_PER_SEC, analyzing_cummul / (double)(analyzing_cummul + collecting_cummul));
	}
	#endif
	
	return cummulative;
    }


    #define sCBS_INTRODUCE_PATH_VERTEX_DELTA_CONFLICTS(upper_node_id)                                              	                                     \
    {								 	                                                                                     \
        Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id);                                                                         \
        next_node_1.m_node_id = search_Store.size();                                                                                                         \
        next_node_1.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
	sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_1.m_node_id;                                                                                \
        search_Store.push_back(next_node_1);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_1.m_node_id, &search_Store));	                                                                     \
        Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id);                                                                         \
        next_node_2.m_node_id = search_Store.size();		    	                                                                                     \
        next_node_2.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
        sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
        search_Store[(upper_node_id)].m_right_node_id = next_node_2.m_node_id;                                                                               \
        search_Store.push_back(next_node_2);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_2.m_node_id, &search_Store));	                                                                     \
    }

    /*
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
    */
    
    
    sInt_32 sCBS::examine_NonconflictingPathsDelta(const sInstance      &instance,
						   sInt_32               upper_node_id,
						   Cooccupations_vector &space_Cooccupations,
						   sInt_32               cost_limit,
						   Nodes_vector         &search_Store,
						   NodeReferences_mset  &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);
	const AgentPaths_vector &agent_Paths = m_delta_agent_Paths;
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		
	if ((cummulative = analyze_NonconflictingPaths(instance, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
	#ifdef sPROFILE
	{	
	    analyzing_end = clock();
	    analyzing_cummul += (analyzing_end - analyzing_begin);
	    
	    collecting_begin = clock();
	}
	#endif		    
	
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
				
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
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
				
				if (*exp_agent != agent_id)
				{
				    Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
				    if (next_collision < principal_collision)
				    {
					principal_collision = next_collision;
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
			
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}
	#ifdef sPROFILE
	{		
	    collecting_end = clock();
	    collecting_cummul += (collecting_end - collecting_begin);
	}
	#endif
	
	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    sCBS_INTRODUCE_PATH_VERTEX_DELTA_CONFLICTS(upper_node_id);
/*	    
	    Node next_node(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id);
	    search_Queue.insert(next_node);

	    next_node = Node(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    search_Queue.insert(next_node);
*/
	    return -1;
	}
        #ifdef sPROFILE
	{		
	    printf("** Times: ana:%.3f col:%.3f (bal:%.3f)\n", analyzing_cummul / (double)CLOCKS_PER_SEC, collecting_cummul / (double)CLOCKS_PER_SEC, analyzing_cummul / (double)(analyzing_cummul + collecting_cummul));
	}
	#endif
	
	return cummulative;
    }                


/*----------------------------------------------------------------------------*/

    sInt_32 sCBS::analyze_NonconflictingPaths(const sInstance           &instance,
					      const AgentPaths_vector   &agent_Paths,
					      sInt_32                   &tanglement) const
    {
	AgentConflicts_vector dummy_Conflicts;
	AgentEdgeConflicts_vector dummy_edge_Conflicts;
	
	return analyze_NonconflictingPaths(instance,
					   dummy_Conflicts,
					   dummy_edge_Conflicts,
					   agent_Paths,
					   tanglement);		
    }

    
    sInt_32 sCBS::analyze_NonconflictingPaths(const sInstance           &instance,
					      AgentConflicts_vector     &sUNUSED(agent_Conflicts),
					      AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
					      const AgentPaths_vector   &agent_Paths,
					      sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;

	sInt_32 cummulative = 0;
	tanglement = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			++tanglement;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    // sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);			    
			    ++tanglement;
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    ++tanglement;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::analyze_NonconflictingPaths(const sInstance           &instance,
					      const AgentPaths_vector   &agent_Paths,
					      Cooccupations_vector      &space_Cooccupations,
					      sInt_32                   &tanglement) const    
    {
	AgentConflicts_vector dummy_Conflicts;
	AgentEdgeConflicts_vector dummy_edge_Conflicts;
	
	return analyze_NonconflictingPaths(instance,
					   dummy_Conflicts,
					   dummy_edge_Conflicts,
					   agent_Paths,
					   space_Cooccupations,
					   tanglement);	
    }

    sInt_32 sCBS::analyze_NonconflictingPaths(const sInstance           &instance,
					      AgentConflicts_vector     &sUNUSED(agent_Conflicts),
					      AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
					      const AgentPaths_vector   &agent_Paths,
					      Cooccupations_vector      &space_Cooccupations,
					      sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	tanglement = 0;

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
				++tanglement;
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
				if (*exp_agent != agent_id)
				{				
				    // sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				    ++tanglement;
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
			    ++tanglement;
			}
		    }
		}
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::update_NonconflictingPaths(sInt_32                    upd_agent_id,
					     const sInstance           &instance,
					     Occupations_vector        &sUNUSED(space_Occupations_),
					     AgentConflicts_vector     &agent_Conflicts,
					     AgentEdgeConflicts_vector &agent_edge_Conflicts,
					     AgentPaths_vector         &agent_Paths,
					     sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	VertexIDs_vector stored_path = agent_Paths[upd_agent_id];
  	agent_Paths[upd_agent_id].clear();

	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(upd_agent_id),
								 instance.m_goal_configuration.get_AgentLocation(upd_agent_id),
								 agent_Conflicts[upd_agent_id],
								 agent_edge_Conflicts[upd_agent_id],
								 agent_Paths[upd_agent_id])) < 0)
	    {
		agent_Paths[upd_agent_id] = stored_path;
		return -1;
	    }
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);
	    }
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}
	if (cummulative > cost_limit)
	{
	    agent_Paths[upd_agent_id] = stored_path;
	    return -1;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
	    
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
	
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPaths(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			agent_Paths[upd_agent_id] = stored_path;
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);
			    
			    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			    {
				return cost;
			    }
			    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			    
			    sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    if ((cost = update_NonconflictingPaths(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			    {
				return cost;
			    }
			    sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
			    agent_Paths[upd_agent_id] = stored_path;
			    return -1;
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
		    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPaths(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPaths(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    agent_Paths[upd_agent_id] = stored_path;
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }	    
	}
	return cummulative;
    }            

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::revise_NonconflictingPermutation(const sInstance           &instance,
						   AgentConflicts_vector     &agent_Conflicts,
						   AgentEdgeConflicts_vector &agent_edge_Conflicts,
						   const AgentPaths_vector   &agent_Paths,
						   sInt_32                    cost_limit,
						   Nodes_mmap                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		       
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
			
			next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);

			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {			    
				Node next_node(agent_id, agent_Conflicts, agent_Paths);
				sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
				search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
				
				next_node = Node(swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				sCBS_ADD_NODE_AGENT_CONFLICT(next_node, swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

				return -1;
			    }
			    */
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
		    
		    next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}

	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingPermutation(const sInstance           &instance,
						   AgentConflicts_vector     &agent_Conflicts,
						   AgentEdgeConflicts_vector &agent_edge_Conflicts,
						   const AgentPaths_vector   &agent_Paths,
						   sInt_32                    cost_limit,
						   Nodes_mset                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}

	if ((cummulative = analyze_NonconflictingPermutation(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, tanglement)) > cost_limit)
	{
	    return -1;
	}	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);
			
			next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);

			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {			    
				Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
				search_Queue.insert(next_node);
				
				next_node = Node(cummulative, tanglement, swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				sCBS_ADD_NODE_AGENT_CONFLICT(next_node, swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				search_Queue.insert(next_node);

				return -1;
			    }
			    */
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingPermutation(const sInstance           &instance,
						   AgentConflicts_vector     &agent_Conflicts,
						   AgentEdgeConflicts_vector &agent_edge_Conflicts,
						   const AgentPaths_vector   &agent_Paths,
						   Cooccupations_vector      &space_Cooccupations,
						   sInt_32                    cost_limit,
						   Nodes_mset                &search_Queue) const
    {
	Collisions_mset agent_Collisions;
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	if ((cummulative = analyze_NonconflictingPermutation(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    return -1;
	}
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
/*
		    if (!agent_Collisions.empty())
		    {
			Collisions_mset::const_iterator leading_collision = agent_Collisions.begin();

			if (occupation_collision != space_Cooccupations[i].end() && leading_collision->m_cooccupation > occupation_collision->second.size())
			{
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				if (*collide_agent > agent_id)
				{
				    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
				}
			    }
			}
		    }
		    else
*/
		    {
			if (occupation_collision != space_Cooccupations[i].end())
			{
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				if (*collide_agent > agent_id)
				{
				    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
				}
			    }
			}
		    }
/*
		    if (!agent_Collisions.empty())
		    {
			Collisions_mset::const_iterator leading_collision = agent_Collisions.begin();
			
			if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
			{
			    Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			    
			    if (swap_expectation_pred != space_Cooccupations[i - 1].end() && leading_collision->m_cooccupation > swap_expectation_pred->second.size()) // move into occupied
			    {
				for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
				{
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				    
				    if (*exp_agent != agent_id)
				    {
					agent_Collisions.insert(Collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]));
				    }
				}
			    }
			}			
		    }
		    else
*/
		    {
			if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
			{
			    Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			    
			    if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied is acceptable
			    {
				/*
				for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
				{
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i - 1);

				    if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				    {				    				    
					if (*exp_agent != agent_id)
					{
					    agent_Collisions.insert(Collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]));
					}
				    }
				}
				*/
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
			    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]));
			}
		    }		    
		}
	    }
	}
	
	if (!agent_Collisions.empty())
	{
	    Collisions_mset::const_iterator principal_collision = agent_Collisions.begin();

	    Node next_node(cummulative, tanglement, principal_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_A_id, principal_collision->m_level_A, principal_collision->m_vertex_A_id);
	    search_Queue.insert(next_node);

	    next_node = Node(cummulative, tanglement, principal_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_B_id, principal_collision->m_level_B, principal_collision->m_vertex_B_id);
	    search_Queue.insert(next_node);

	    return -1;
	}       			
	return cummulative;
    }

    
    sInt_32 sCBS::examine_NonconflictingPermutation(const sInstance           &instance,
						    AgentConflicts_vector     &agent_Conflicts,
						    AgentEdgeConflicts_vector &agent_edge_Conflicts,
						    const AgentPaths_vector   &agent_Paths,
						    Cooccupations_vector      &space_Cooccupations,
						    sInt_32                    cost_limit,
						    Nodes_mset                &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);	
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		
	if ((cummulative = analyze_NonconflictingPermutation(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
	#ifdef sPROFILE
	{	
	    analyzing_end = clock();
	    analyzing_cummul += (analyzing_end - analyzing_begin);
	    
	    collecting_begin = clock();
	}
	#endif		    
	
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
				
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i - 1);

				if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				{
				    if (*exp_agent != agent_id)
				    {
					Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
					if (next_collision < principal_collision)
					{
					    principal_collision = next_collision;
					}
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
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}
	#ifdef sPROFILE
	{		
	    collecting_end = clock();
	    collecting_cummul += (collecting_end - collecting_begin);
	}
	#endif
	
	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    Node next_node(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id);
	    search_Queue.insert(next_node);

	    next_node = Node(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    search_Queue.insert(next_node);

	    return -1;
	}
        #ifdef sPROFILE
	{		
	    printf("** Times: ana:%.3f col:%.3f (bal:%.3f)\n", analyzing_cummul / (double)CLOCKS_PER_SEC, collecting_cummul / (double)CLOCKS_PER_SEC, analyzing_cummul / (double)(analyzing_cummul + collecting_cummul));
	}
	#endif
	
	return cummulative;
    }            


    #define sCBS_INTRODUCE_PERMUTATION_VERTEX_DELTA_CONFLICTS(upper_node_id)                                              	                             \
    {								 	                                                                                     \
        Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id);                                                                         \
        next_node_1.m_node_id = search_Store.size();                                                                                                         \
        next_node_1.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
	sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_1.m_node_id;                                                                                \
        search_Store.push_back(next_node_1);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_1.m_node_id, &search_Store));	                                                                     \
        Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id);                                                                         \
        next_node_2.m_node_id = search_Store.size();		    	                                                                                     \
        next_node_2.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
        sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
        search_Store[(upper_node_id)].m_right_node_id = next_node_2.m_node_id;                                                                               \
        search_Store.push_back(next_node_2);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_2.m_node_id, &search_Store));	                                                                     \
    }

    /*
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
    */

    
    sInt_32 sCBS::examine_NonconflictingPermutationDelta(const sInstance      &instance,
							 sInt_32               upper_node_id,
							 Cooccupations_vector &space_Cooccupations,
							 sInt_32               cost_limit,
							 Nodes_vector         &search_Store,
							 NodeReferences_mset  &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);
	const AgentPaths_vector &agent_Paths = m_delta_agent_Paths;
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		
	if ((cummulative = analyze_NonconflictingPaths(instance, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
	#ifdef sPROFILE
	{	
	    analyzing_end = clock();
	    analyzing_cummul += (analyzing_end - analyzing_begin);
	    
	    collecting_begin = clock();
	}
	#endif		    
	
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
				
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
			    }
			}
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied
			{
			    /*
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i - 1);
				
				if (*exp_agent != agent_id)
				{
				    Collision next_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]);
				    
				    if (next_collision < principal_collision)
				    {
					principal_collision = next_collision;
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
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    Collision next_collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]);
			
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}
	#ifdef sPROFILE
	{		
	    collecting_end = clock();
	    collecting_cummul += (collecting_end - collecting_begin);
	}
	#endif
	
	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    sCBS_INTRODUCE_PERMUTATION_VERTEX_DELTA_CONFLICTS(upper_node_id);
/*	    
	    Node next_node(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id);
	    search_Queue.insert(next_node);

	    next_node = Node(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
	    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	    search_Queue.insert(next_node);
*/
	    return -1;
	}
        #ifdef sPROFILE
	{		
	    printf("** Times: ana:%.3f col:%.3f (bal:%.3f)\n", analyzing_cummul / (double)CLOCKS_PER_SEC, collecting_cummul / (double)CLOCKS_PER_SEC, analyzing_cummul / (double)(analyzing_cummul + collecting_cummul));
	}
	#endif
	
	return cummulative;
    }                

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::analyze_NonconflictingPermutation(const sInstance           &instance,
						    AgentConflicts_vector     &sUNUSED(agent_Conflicts),
						    AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
						    const AgentPaths_vector   &agent_Paths,
						    sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;

	sInt_32 cummulative = 0;
	tanglement = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			++tanglement;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i);
			    
			    if (agent_Paths[swap_expectation_pred->second][ii] == agent_Paths[agent_id][i - 1])
			    {
				++tanglement;				
			    }
			    */
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    ++tanglement;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::analyze_NonconflictingPermutation(const sInstance           &instance,
						    AgentConflicts_vector     &sUNUSED(agent_Conflicts),
						    AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
						    const AgentPaths_vector   &agent_Paths,
						    Cooccupations_vector      &space_Cooccupations,
						    sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	tanglement = 0;

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
				++tanglement;
			    }
			}
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
			    {
				if (*exp_agent != agent_id)
				{				
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);

				    if (agent_Paths[*exp_agent][ii] == agent_Paths[agent_id][i - 1])
				    {				    
					++tanglement;
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
		Cooccupation_umap::const_iterator occupation_collision = space_Cooccupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Cooccupations[i].end())
		{
		    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
		    {
			if (*collide_agent > agent_id)
			{
			    ++tanglement;
			}
		    }
		}
	    }
	}
	return cummulative;
    }

    
    sInt_32 sCBS::update_NonconflictingPermutation(sInt_32                    upd_agent_id,
						   const sInstance           &instance,
						   Occupations_vector        &sUNUSED(space_Occupations_),
						   AgentConflicts_vector     &agent_Conflicts,
						   AgentEdgeConflicts_vector &agent_edge_Conflicts,
						   AgentPaths_vector         &agent_Paths,
						   sInt_32                    cost_limit) const
    {
	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	VertexIDs_vector stored_path = agent_Paths[upd_agent_id];
  	agent_Paths[upd_agent_id].clear();

	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(upd_agent_id),
								 instance.m_goal_configuration.get_AgentLocation(upd_agent_id),
								 agent_Conflicts[upd_agent_id],
								 agent_edge_Conflicts[upd_agent_id],
								 agent_Paths[upd_agent_id])) < 0)
	    {
		agent_Paths[upd_agent_id] = stored_path;
		return -1;
	    }
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);
	    }
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}
	if (cummulative > cost_limit)
	{
	    agent_Paths[upd_agent_id] = stored_path;
	    return -1;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
	    
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
	
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingPermutation(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			agent_Paths[upd_agent_id] = stored_path;
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied is acceptable
			{
			    /*
			    sInt_32 ii = sMIN(agent_Paths[swap_expectation_pred->second].size() - 1, i - 1);

			    if (agent_Paths[swap_expectation_pred->second][ii] != agent_Paths[agent_id][i - 1])
			    {			    			    
				sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
				
				sCBS_ADD_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				if ((cost = update_NonconflictingPermutation(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				{
				    return cost;
				}
				sCBS_DEL_AGENT_CONFLICT(swap_expectation_pred->second, ii, agent_Paths[swap_expectation_pred->second][ii]);
				agent_Paths[upd_agent_id] = stored_path;
				return -1;
			    }
			    */
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
		    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPermutation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingPermutation(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    agent_Paths[upd_agent_id] = stored_path;
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }	    
	}
	return cummulative;
    }            


/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::revise_NonconflictingRotation(const sInstance           &instance,
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						const AgentPaths_vector   &agent_Paths,
						sInt_32                    cost_limit,
						Nodes_mmap                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	    
	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		       
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
			
			next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    if (i < agent_Paths[swap_expectation_pred->second].size())
			    {
				if (agent_Paths[swap_expectation_pred->second][i] == agent_Paths[agent_id][i - 1])
				{			    
				    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, agent_id, i-1, agent_Paths[agent_id][i-1], agent_Paths[agent_id][i]);
				    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
				
				    next_node = Node(swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, swap_expectation_pred->second, i, agent_Paths[swap_expectation_pred->second][i-1], agent_Paths[swap_expectation_pred->second][i]);
				    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

				    return -1;
				}
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));
		    
		    next_node = Node(occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(Nodes_mmap::value_type(cummulative, next_node));

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}

	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingRotation(const sInstance           &instance,
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						const AgentPaths_vector   &agent_Paths,
						sInt_32                    cost_limit,
						Nodes_mset                &search_Queue) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();	
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}

	if ((cummulative = analyze_NonconflictingRotation(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, tanglement)) > cost_limit)
	{
	    return -1;
	}	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);
			
			next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
			sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][i]);
			search_Queue.insert(next_node);

			return -1;			
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    if (i < agent_Paths[swap_expectation_pred->second].size())
			    {
				if (agent_Paths[swap_expectation_pred->second][i] == agent_Paths[agent_id][i - 1])
				{			    
				    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, agent_id, i-1, agent_Paths[agent_id][i-1], agent_Paths[agent_id][i]);
				    search_Queue.insert(next_node);
				    
				    next_node = Node(cummulative, tanglement, swap_expectation_pred->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
				    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, swap_expectation_pred->second, i-1, agent_Paths[swap_expectation_pred->second][i-1], agent_Paths[swap_expectation_pred->second][i]);
				    search_Queue.insert(next_node);

				    return -1;
				}
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    Node next_node(cummulative, tanglement, agent_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, occupation_collision->second, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    search_Queue.insert(next_node);

		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::revise_NonconflictingRotation(const sInstance           &instance,
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						const AgentPaths_vector   &agent_Paths,
						Cooccupations_vector      &space_Cooccupations,
						sInt_32                    cost_limit,
						Nodes_mset                &search_Queue) const
    {
	Collisions_mset agent_Collisions;
	EdgeCollisions_mset agent_edge_Collisions;
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	if ((cummulative = analyze_NonconflictingRotation(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    return -1;
	}
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
/*
		    if (!agent_Collisions.empty())
		    {
			Collisions_mset::const_iterator leading_collision = agent_Collisions.begin();

			if (occupation_collision != space_Cooccupations[i].end() && leading_collision->m_cooccupation > occupation_collision->second.size())
			{
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				if (*collide_agent > agent_id)
				{
				    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
				}
			    }
			}
		    }
		    else
*/
		    {
			if (occupation_collision != space_Cooccupations[i].end())
			{
			    for (AgentIDs_uset::const_iterator collide_agent = occupation_collision->second.begin(); collide_agent != occupation_collision->second.end(); ++collide_agent)
			    {
				if (*collide_agent > agent_id)
				{
				    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][i]));
				}
			    }
			}
		    }
/*
		    if (!agent_Collisions.empty())
		    {
			Collisions_mset::const_iterator leading_collision = agent_Collisions.begin();
			
			if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
			{
			    Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			    
			    if (swap_expectation_pred != space_Cooccupations[i - 1].end() && leading_collision->m_cooccupation > swap_expectation_pred->second.size()) // move into occupied
			    {
				for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
				{
				    sInt_32 ii = sMIN(agent_Paths[*exp_agent].size() - 1, i);
				    
				    if (*exp_agent != agent_id)
				    {
					agent_Collisions.insert(Collision(swap_expectation_pred->second.size(), agent_id, *exp_agent, i, ii, agent_Paths[agent_id][i], agent_Paths[*exp_agent][ii]));
				    }
				}
			    }
			}			
		    }
		    else
*/
		    {
			if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
			{
			    Cooccupation_umap::const_iterator swap_expectation_pred = space_Cooccupations[i - 1].find(agent_Paths[agent_id][i]);
			    
			    if (swap_expectation_pred != space_Cooccupations[i - 1].end()) // move into occupied
			    {
				for (AgentIDs_uset::const_iterator exp_agent = swap_expectation_pred->second.begin(); exp_agent != swap_expectation_pred->second.end(); ++exp_agent)
				{
				    if (i < agent_Paths[*exp_agent].size())
				    {
					if (agent_Paths[*exp_agent][i] == agent_Paths[agent_id][i - 1])
					{				    				    
					    if (*exp_agent != agent_id)
					    {
						agent_edge_Collisions.insert(EdgeCollision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
											   i-1,
											   agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
											   agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]));
					    }
					}
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
			    agent_Collisions.insert(Collision(occupation_collision->second.size(), agent_id, *collide_agent, i, agent_Paths[agent_id][agent_path_length - 1]));
			}
		    }		    
		}
	    }
	}
	
	if (!agent_Collisions.empty())
	{
	    Collisions_mset::const_iterator principal_collision = agent_Collisions.begin();
	    if (!agent_edge_Collisions.empty())
	    {
		EdgeCollisions_mset::const_iterator principal_edge_collision = agent_edge_Collisions.begin();

		if (*principal_edge_collision < *principal_collision)
		{
		    Node next_node(cummulative, tanglement, principal_edge_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_A_id, principal_edge_collision->m_level_A, principal_edge_collision->m_edge_A_u_id, principal_edge_collision->m_edge_A_v_id);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, principal_edge_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_B_id, principal_edge_collision->m_level_B, principal_edge_collision->m_edge_B_u_id, principal_edge_collision->m_edge_B_v_id);	    
		    search_Queue.insert(next_node);	    		    
		    return -1;		    
		}
		else
		{
		    Node next_node(cummulative, tanglement, principal_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_A_id, principal_collision->m_level_A, principal_collision->m_vertex_A_id);
		    search_Queue.insert(next_node);
		    
		    next_node = Node(cummulative, tanglement, principal_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_B_id, principal_collision->m_level_B, principal_collision->m_vertex_B_id);
		    search_Queue.insert(next_node);		    
		    return -1;
		}
	    }
	    else
	    {
		Node next_node(cummulative, tanglement, principal_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_A_id, principal_collision->m_level_A, principal_collision->m_vertex_A_id);
		search_Queue.insert(next_node);
		
		next_node = Node(cummulative, tanglement, principal_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_CONFLICT(next_node, principal_collision->m_agent_B_id, principal_collision->m_level_B, principal_collision->m_vertex_B_id);
		search_Queue.insert(next_node);		    
		return -1;		
	    }
	}
	else
	{
	    if (!agent_edge_Collisions.empty())
	    {
		EdgeCollisions_mset::const_iterator principal_edge_collision = agent_edge_Collisions.begin();

		Node next_node(cummulative, tanglement, principal_edge_collision->m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_A_id, principal_edge_collision->m_level_A, principal_edge_collision->m_edge_A_u_id, principal_edge_collision->m_edge_A_v_id);
		search_Queue.insert(next_node);
	    
		next_node = Node(cummulative, tanglement, principal_edge_collision->m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node, principal_edge_collision->m_agent_B_id, principal_edge_collision->m_level_B, principal_edge_collision->m_edge_B_u_id, principal_edge_collision->m_edge_B_v_id);	    
		search_Queue.insert(next_node);	    		
		return -1;
	    }
	}

	return cummulative;
    }

    
    sInt_32 sCBS::examine_NonconflictingRotation(const sInstance           &instance,
						 AgentConflicts_vector     &agent_Conflicts,
						 AgentEdgeConflicts_vector &agent_edge_Conflicts,
						 const AgentPaths_vector   &agent_Paths,
						 Cooccupations_vector      &space_Cooccupations,
						 sInt_32                    cost_limit,
						 Nodes_mset                &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);
	EdgeCollision principal_edge_collision(sINT_32_MAX, 1, 1, 0, 0, 0, 0, 0);

        #ifdef sDEBUG
	{
	    printf("----> Begin <----\n");
	    
	    sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		printf("Agnt %d: ", agent_id);
		
		for (sInt_32 i = 0; i < agent_Paths[agent_id].size(); ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }

	    sCBS_SHOW_CONFLICTS(agent_Conflicts);
	    sCBS_SHOW_EDGE_CONFLICTS(agent_edge_Conflicts);

	    printf("----> End <----\n");
	}
        #endif
		    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		
	if ((cummulative = analyze_NonconflictingRotation(instance, agent_Conflicts, agent_edge_Conflicts, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
	#ifdef sPROFILE
	{	
	    analyzing_end = clock();
	    analyzing_cummul += (analyzing_end - analyzing_begin);
	    
	    collecting_begin = clock();
	}
	#endif		    
	
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
			
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
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
				if (i < agent_Paths[*exp_agent].size())
				{
				    if (agent_Paths[*exp_agent][i] == agent_Paths[agent_id][i-1])
				    {
					if (*exp_agent != agent_id)
					{
					    EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									      i-1,
									      agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									      agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					    
					    if (next_edge_collision < principal_edge_collision)
					    {
						principal_edge_collision = next_edge_collision;
					    }
					}
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
			
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }
		}
	    }
	}
	#ifdef sPROFILE
	{		
	    collecting_end = clock();
	    collecting_cummul += (collecting_end - collecting_begin);
	}
	#endif

	#ifdef sDEBUG
	{
	    printf("pc: %d,%d,%d %d,%d,%d\n",
		   principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id,
		   principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
	}
	#endif

        #ifdef sDEBUG
	{
	    sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		printf("Agnt %d: ", agent_id);
		
		for (sInt_32 i = 0; i < agent_Paths[agent_id].size(); ++i)
		{
		    printf("%d ", agent_Paths[agent_id][i]);
		}
		printf("\n");
	    }
	}
        #endif

	#ifdef sDEBUG
	{
	    printf("conflicts A:\n");
	    
	    for (sInt_32 level = 0; level < agent_Conflicts[principal_collision.m_agent_A_id].size(); ++level)
	    {
		printf("%d: ", level);
		for (VertexIDs_uset::const_iterator conf = agent_Conflicts[principal_collision.m_agent_A_id][level].begin(); conf != agent_Conflicts[principal_collision.m_agent_A_id][level].end(); ++conf)
		{
		    printf("%d ", *conf);
		}
		printf("\n");
	    }

	    printf("conflicts B:\n");
	    
	    for (sInt_32 level = 0; level < agent_Conflicts[principal_collision.m_agent_B_id].size(); ++level)
	    {
		printf("%d: ", level);
		for (VertexIDs_uset::const_iterator conf = agent_Conflicts[principal_collision.m_agent_B_id][level].begin(); conf != agent_Conflicts[principal_collision.m_agent_B_id][level].end(); ++conf)
		{
		    printf("%d ", *conf);
		}
		printf("\n");
	    }	    
	}
	#endif

	sInt_32 cummulative_result = cummulative;

	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		if (principal_edge_collision < principal_collision)
		{
		    Node next_node_1(cummulative, tanglement, principal_edge_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id);
		    search_Queue.insert(next_node_1);	    
	    
		    Node next_node_2(cummulative, tanglement, principal_edge_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id);
		    search_Queue.insert(next_node_2);

		    cummulative_result = -1;
		}
		else
		{
		    Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id);
		    search_Queue.insert(next_node_1);	    
		    
		    Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		    sCBS_ADD_NODE_AGENT_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
		    search_Queue.insert(next_node_2);

		    cummulative_result = -1;		    
		}
	    }
	    else
	    {
		Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id);
		search_Queue.insert(next_node_1);
		
		Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id);
		search_Queue.insert(next_node_2);

		cummulative_result = -1;
	    }
	}
	else
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		Node next_node_1(cummulative, tanglement, principal_edge_collision.m_agent_A_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id);
		search_Queue.insert(next_node_1);	    
		
		Node next_node_2(cummulative, tanglement, principal_edge_collision.m_agent_B_id, agent_Conflicts, agent_edge_Conflicts, agent_Paths);
		sCBS_ADD_NODE_AGENT_EDGE_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id);
		search_Queue.insert(next_node_2);
		    
		cummulative_result = -1;
	    }
	}
        #ifdef sPROFILE
	{		
	    printf("** Times: ana:%.3f col:%.3f (bal:%.3f)\n", analyzing_cummul / (double)CLOCKS_PER_SEC, collecting_cummul / (double)CLOCKS_PER_SEC, analyzing_cummul / (double)(analyzing_cummul + collecting_cummul));
	}
	#endif
	
	return cummulative_result;
    }


    #define sCBS_INTRODUCE_ROTATION_VERTEX_DELTA_CONFLICTS(upper_node_id)                                              	                                     \
    {								 	                                                                                     \
        Node next_node_1(cummulative, tanglement, principal_collision.m_agent_A_id);                                                                         \
        next_node_1.m_node_id = search_Store.size();                                                                                                         \
        next_node_1.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
	sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_1.m_node_id;                                                                                \
        search_Store.push_back(next_node_1);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_1.m_node_id, &search_Store));	                                                                     \
        Node next_node_2(cummulative, tanglement, principal_collision.m_agent_B_id);                                                                         \
        next_node_2.m_node_id = search_Store.size();		    	                                                                                     \
        next_node_2.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                               \
        sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
        search_Store[(upper_node_id)].m_right_node_id = next_node_2.m_node_id;                                                                               \
        search_Store.push_back(next_node_2);                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_2.m_node_id, &search_Store));	                                                                     \
    }

    /*
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_1, principal_collision.m_agent_A_id, principal_collision.m_level_A, principal_collision.m_vertex_A_id); \
            sCBS_ADD_NODE_AGENT_DELTA_CONFLICT(next_node_2, principal_collision.m_agent_B_id, principal_collision.m_level_B, principal_collision.m_vertex_B_id); \
    */
    
    #define sCBS_INTRODUCE_ROTATION_EDGE_DELTA_CONFLICTS(upper_node_id)                                                                                                                                           \
    {                                                                                                                                                                                                                    \
        Node next_node_1(cummulative, tanglement, principal_edge_collision.m_agent_A_id);                                                                                                                                \
        next_node_1.m_node_id = search_Store.size();                                                                                                                                                                     \
        next_node_1.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                                                                                           \
        sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_1.m_node_id;                                                                                                                                            \
        search_Store.push_back(next_node_1);				                                                                                                                                                 \
        search_Queue.insert(NodeReference(next_node_1.m_node_id, &search_Store));	                                                                                                                                 \
        Node next_node_2(cummulative, tanglement, principal_edge_collision.m_agent_B_id);                                                                                                                                \
        next_node_2.m_node_id = search_Store.size();                                                                                                                                                                     \
        next_node_2.m_upper_node_id = search_Store[(upper_node_id)].m_node_id;                                                                                                                                           \
        sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id); \
        search_Store[(upper_node_id)].m_left_node_id = next_node_2.m_node_id;                                                                                                                                            \
        search_Store.push_back(next_node_2);                                                                                                                                                                             \
        search_Queue.insert(NodeReference(next_node_2.m_node_id, &search_Store));	                                                                                                                                 \
    }

    /*
            sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_1, principal_edge_collision.m_agent_A_id, principal_edge_collision.m_level_A, principal_edge_collision.m_edge_A_u_id, principal_edge_collision.m_edge_A_v_id); \
        sCBS_ADD_NODE_AGENT_EDGE_DELTA_CONFLICT(next_node_2, principal_edge_collision.m_agent_B_id, principal_edge_collision.m_level_B, principal_edge_collision.m_edge_B_u_id, principal_edge_collision.m_edge_B_v_id); \    
    */
    
    sInt_32 sCBS::examine_NonconflictingRotationDelta(const sInstance      &instance,
						      sInt_32               upper_node_id,
						      Cooccupations_vector &space_Cooccupations,
						      sInt_32               cost_limit,
						      Nodes_vector         &search_Store,
						      NodeReferences_mset  &search_Queue) const
    {
	Collision principal_collision(sINT_32_MAX, 1, 1, 0, 0);
	EdgeCollision principal_edge_collision(sINT_32_MAX, 1, 1, 0, 0, 0, 0, 0);
	
	const AgentPaths_vector &agent_Paths = m_delta_agent_Paths;
	    
	sInt_32 agent_path_length;
	sInt_32 cummulative, tanglement;

	#ifdef sPROFILE
	{
	    analyzing_begin = clock();
	}
	#endif
			
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	cummulative = 0;
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    if (cummulative > cost_limit)
	    {
		return -1;
	    }
	}		
	if ((cummulative = analyze_NonconflictingPaths(instance, agent_Paths, space_Cooccupations, tanglement)) > cost_limit)
	{
	    sASSERT(false);
	}
	#ifdef sPROFILE
	{	
	    analyzing_end = clock();
	    analyzing_cummul += (analyzing_end - analyzing_begin);
	    
	    collecting_begin = clock();
	}
	#endif		    

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
			
				if (next_collision < principal_collision)
				{
				    principal_collision = next_collision;
				}
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
				if (i < agent_Paths[*exp_agent].size())
				{
				    if (agent_Paths[*exp_agent][i] == agent_Paths[agent_id][i-1])
				    {
					if (*exp_agent != agent_id)
					{
					    EdgeCollision next_edge_collision(swap_expectation_pred->second.size(), agent_id, *exp_agent,
									      i-1,
									      agent_Paths[agent_id][i-1], agent_Paths[agent_id][i],
									      agent_Paths[*exp_agent][i-1], agent_Paths[*exp_agent][i]);
					    
					    if (next_edge_collision < principal_edge_collision)
					    {
						principal_edge_collision = next_edge_collision;
					    }
					}
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
			
			    if (next_collision < principal_collision)
			    {
				principal_collision = next_collision;
			    }			    
			}
		    }		    
		}
	    }
	}
	#ifdef sPROFILE
	{		
	    collecting_end = clock();
	    collecting_cummul += (collecting_end - collecting_begin);
	}
	#endif

	sInt_32 cummulative_result = cummulative;

	if (principal_collision.m_cooccupation < sINT_32_MAX)
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		if (principal_edge_collision < principal_collision)
		{
		    sCBS_INTRODUCE_ROTATION_EDGE_DELTA_CONFLICTS(upper_node_id);
		    cummulative_result = -1;
		}
		else
		{
		    sCBS_INTRODUCE_ROTATION_VERTEX_DELTA_CONFLICTS(upper_node_id);
		    cummulative_result = -1;		    
		}
	    }
	    else
	    {
		sCBS_INTRODUCE_ROTATION_VERTEX_DELTA_CONFLICTS(upper_node_id);
		cummulative_result = -1;
	    }
	}
	else
	{
	    if (principal_edge_collision.m_cooccupation < sINT_32_MAX)
	    {
		sCBS_INTRODUCE_ROTATION_EDGE_DELTA_CONFLICTS(upper_node_id);	    
		cummulative_result = -1;
	    }
	}

        #ifdef sPROFILE
	{		
	    printf("** Times: ana:%.3f col:%.3f (bal:%.3f)\n", analyzing_cummul / (double)CLOCKS_PER_SEC, collecting_cummul / (double)CLOCKS_PER_SEC, analyzing_cummul / (double)(analyzing_cummul + collecting_cummul));
	}
	#endif
	
	return cummulative_result;
    }


/*----------------------------------------------------------------------------*/
    
    sInt_32 sCBS::analyze_NonconflictingRotation(const sInstance           &instance,
						 AgentConflicts_vector     &sUNUSED(agent_Conflicts),
						 AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
						 const AgentPaths_vector   &agent_Paths,
						 sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	Occupations_vector space_Occupations;

	sInt_32 cummulative = 0;
	tanglement = 0;

	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();

	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);		
	    }
	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
		
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
		    
		    if (occupation_collision != space_Occupations[i].end())
		    {
			++tanglement;
		    }
		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    if (i < agent_Paths[swap_expectation_pred->second].size())
			    {
				if (agent_Paths[swap_expectation_pred->second][i] == agent_Paths[agent_id][i - 1])
				{
				    ++tanglement;				
				}
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
	    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		if (occupation_collision != space_Occupations[i].end())
		{
		    ++tanglement;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::analyze_NonconflictingRotation(const sInstance           &instance,
						 AgentConflicts_vector     &sUNUSED(agent_Conflicts),
						 AgentEdgeConflicts_vector &sUNUSED(agent_edge_Conflicts),
						 const AgentPaths_vector   &agent_Paths,
						 Cooccupations_vector      &space_Cooccupations,
						 sInt_32                   &tanglement) const
    {
	sInt_32 agent_path_length;
	tanglement = 0;

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
				++tanglement;
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
				if (*exp_agent != agent_id)
				{				
				    if (i < agent_Paths[*exp_agent].size())
				    {
					if (agent_Paths[*exp_agent][i] == agent_Paths[agent_id][i - 1])
					{				    
					    ++tanglement;
					}
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
			    ++tanglement;
			}
		    }
		}
	    }
	}
	return cummulative;
    }


    sInt_32 sCBS::update_NonconflictingRotation(sInt_32                    upd_agent_id,
						const sInstance           &instance,
						Occupations_vector        &sUNUSED(space_Occupations_),
						AgentConflicts_vector     &agent_Conflicts,
						AgentEdgeConflicts_vector &agent_edge_Conflicts,
						AgentPaths_vector         &agent_Paths,
						sInt_32                    cost_limit) const
    {
	sASSERT(false);

	sInt_32 cost, cummulative, agent_path_length;
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();
	
	VertexIDs_vector stored_path = agent_Paths[upd_agent_id];
  	agent_Paths[upd_agent_id].clear();

	Occupations_vector space_Occupations;
	cummulative = 0;

	#ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	}
	#endif

	{	
	    if ((agent_path_length = find_NonconflictingSequence(instance.m_environment,
								 instance.m_start_configuration.get_AgentLocation(upd_agent_id),
								 instance.m_goal_configuration.get_AgentLocation(upd_agent_id),
								 agent_Conflicts[upd_agent_id],
								 agent_edge_Conflicts[upd_agent_id],
								 agent_Paths[upd_agent_id])) < 0)
	    {
		agent_Paths[upd_agent_id] = stored_path;
		return -1;
	    }
	}
	
	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    agent_path_length = agent_Paths[agent_id].size();
	    
	    if (!space_Occupations.empty())
	    {
		while (space_Occupations.size() < agent_path_length)
		{
		    space_Occupations.push_back(space_Occupations.back());
		}
	    }
	    else
	    {
		space_Occupations.resize(agent_path_length);
	    }
	    cummulative += (agent_path_length > 1) ? agent_path_length : 0;

	    space_Occupations[0][agent_Paths[agent_id][0]] = agent_id;
	}
	if (cummulative > cost_limit)
	{
	    agent_Paths[upd_agent_id] = stored_path;
	    return -1;
	}
	
	for (sInt_32 i = 1;; ++i)
	{
	    bool finished = true;
	    
	    for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	    {
		agent_path_length = agent_Paths[agent_id].size();
		
		if (i < agent_path_length)
		{
		    finished = false;
		    Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][i]);
	
		    if (occupation_collision != space_Occupations[i].end())
		    {
			sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][i]);
		    
			sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			if ((cost = update_NonconflictingRotation(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
			{
			    return cost;
			}
			sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][i]);
			agent_Paths[upd_agent_id] = stored_path;
			return -1;
		    }

		    if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i]) // proper move
		    {
			Occupation_umap::const_iterator swap_expectation_pred = space_Occupations[i - 1].find(agent_Paths[agent_id][i]);		    
			
			if (swap_expectation_pred != space_Occupations[i - 1].end()) // move into occupied
			{
			    if (i < agent_Paths[swap_expectation_pred->second].size())
			    {
				if (agent_Paths[swap_expectation_pred->second][i] == agent_Paths[agent_id][i - 1])
				{			    			    
				    sCBS_ADD_AGENT_EDGE_CONFLICT(agent_id, i-1, agent_Paths[agent_id][i-1], agent_Paths[agent_id][i]);
				    if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				    {
					return cost;
				    }
				    sCBS_DEL_AGENT_EDGE_CONFLICT(agent_id, i-1, agent_Paths[agent_id][i-1], agent_Paths[agent_id][i]);
				    
				    sCBS_ADD_AGENT_EDGE_CONFLICT(swap_expectation_pred->second, i-1, agent_Paths[swap_expectation_pred->second][i-1], agent_Paths[swap_expectation_pred->second][i]);
				    if ((cost = update_NonconflictingRotation(swap_expectation_pred->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
				    {
					return cost;
				    }
				    sCBS_DEL_AGENT_EDGE_CONFLICT(swap_expectation_pred->second, i-1, agent_Paths[swap_expectation_pred->second][i-1], agent_Paths[swap_expectation_pred->second][i]);
				    agent_Paths[upd_agent_id] = stored_path;
				    return -1;
				}
			    }
			}
		    }
		    space_Occupations[i][agent_Paths[agent_id][i]] = agent_id;
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
		    
	    for (sInt_32 i = agent_path_length; i < space_Occupations.size(); ++i)
	    {
		Occupation_umap::const_iterator occupation_collision = space_Occupations[i].find(agent_Paths[agent_id][agent_path_length - 1]);
		
		if (occupation_collision != space_Occupations[i].end())
		{
		    sCBS_ADD_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingRotation(agent_id, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(agent_id, i, agent_Paths[agent_id][agent_path_length - 1]);
		    
		    sCBS_ADD_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    if ((cost = update_NonconflictingRotation(occupation_collision->second, instance, space_Occupations, agent_Conflicts, agent_edge_Conflicts, agent_Paths, cost_limit)) >= 0)
		    {
			return cost;
		    }
		    sCBS_DEL_AGENT_CONFLICT(occupation_collision->second, i, agent_Paths[agent_id][agent_path_length - 1]);
		    agent_Paths[upd_agent_id] = stored_path;
		    return -1;
		}
		space_Occupations[i][agent_Paths[agent_id][agent_path_length - 1]] = agent_id;
	    }	    
	}
	return cummulative;
    }            
   
    
/*----------------------------------------------------------------------------*/

    sInt_32 sCBS::find_NonconflictingSequence(const sUndirectedGraph &graph,
					      sInt_32                 source_id,
					      sInt_32                 sink_id,
					      const Conflicts_vector &Conflicts,					      
					      VertexIDs_vector       &Path) const
    {
	/*
	#ifdef sDEBUG
	{
	    printf("seq:%d -> %d\n", source_id, sink_id);
	}
	#endif
	*/
		
	Visits_list visit_Queue;
	Visits_vector visited_Vertices;

	/*
	#ifdef sDEBUG
	{
	    printf("conflicts:\n");
	    
	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (VertexIDs_uset::const_iterator conf = Conflicts[level].begin(); conf != Conflicts[level].end(); ++conf)
		{
		    printf("%d ", *conf);
		}
		printf("\n");
	    }
	}
	#endif
	*/

	if (source_id == sink_id)
	{
	    bool non_conflicting_sink = true;

	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		if (Conflicts[level].find(sink_id) != Conflicts[level].end())
		{
		    non_conflicting_sink = false;
		    break;
		}
	    }

	    if (non_conflicting_sink)
	    {
		Path.push_back(sink_id);
		return 1;
	    }
	}
  
	if (Conflicts.empty() || Conflicts[0].find(source_id) == Conflicts[0].end())
	{
	    visit_Queue.push_back(Visit(0, source_id, -1));
	    visited_Vertices.push_back(Visits_umap());
	    
	    Visit source_visit(0, source_id, -1);
	    visited_Vertices[0][source_id] = source_visit;
	}
	else
	{
	    return -1;
	}
	
	while (!visit_Queue.empty())
	{
	    const Visit &front_visit = visit_Queue.front();
            /*
	    #ifdef sDEBUG
	    {
		printf("  v:%d,%d,%d\n", front_visit.m_level, front_visit.m_vertex_id, front_visit.m_previous_id);
	    }
	    #endif
	    */

	    if (visited_Vertices.size() <= front_visit.m_level + 1)
	    {
		visited_Vertices.push_back(Visits_umap());
	    }
	    for (sVertex::Neighbors_list::const_iterator neighbor = graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.begin(); neighbor != graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (visited_Vertices.size() <= front_visit.m_level + 1)
		{
		    sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
		    visited_Vertices.push_back(Visits_umap());
		}
		Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];

		if (next_visited_Vertices.find(neighbor_id) == next_visited_Vertices.end())
		{
		    /*
		    #ifdef sDEBUG
		    {
			printf("    n:%d\n", neighbor_id);
		    }
		    #endif
		    */
		    

		    if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(neighbor_id) == Conflicts[front_visit.m_level + 1].end())
		    {
			/*
                        #ifdef sDEBUG
			{
			    printf("      *\n");
			}
                        #endif
			*/
			
			Visit neighbor_visit(front_visit.m_level + 1, neighbor_id, front_visit.m_vertex_id);
			next_visited_Vertices[neighbor_id] = neighbor_visit;

			if (neighbor_id == sink_id)
			{
			    bool non_conflicting_sink = true;

			    for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
			    {
				if (Conflicts[level].find(sink_id) != Conflicts[level].end())
				{
				    non_conflicting_sink = false;
				    break;
				}
			    }

			    if (non_conflicting_sink)
			    {
				sInt_32 prev_vertex_id = sink_id;
				Path.resize(front_visit.m_level + 2, -1);
/*
				printf("Path:\n");
				for (sInt_32 i = 0; i <= front_visit.m_level + 1; ++i)
				{
				    for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
				    {
					printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
				    }
				    printf("\n");
				}
*/			    
				for (sInt_32 i = front_visit.m_level + 1; i >= 0; --i)
				{
				    Path[i] = prev_vertex_id;
				    Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
				    
				    sASSERT(prev_visit != visited_Vertices[i].end());
				    
				    prev_vertex_id = prev_visit->second.m_previous_id;
				}
				return front_visit.m_level + 2;
			    }
			}
			visit_Queue.push_back(neighbor_visit);
		    }
		}
	    }
	    { /* wait action */

		/*
                #ifdef sDEBUG
		{
		    printf("    -:%d\n", front_visit.m_vertex_id);
		}
                #endif		
		*/
		
		if (visited_Vertices.size() <= front_visit.m_level + 1)
		{
		    sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
		    visited_Vertices.push_back(Visits_umap());
		}		
		Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];

		if (next_visited_Vertices.find(front_visit.m_vertex_id) == next_visited_Vertices.end())
		{
		    if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(front_visit.m_vertex_id) == Conflicts[front_visit.m_level + 1].end())
		    {
			/*
                        #ifdef sDEBUG
			{
			    printf("      *\n");
			}
                        #endif
			*/		
			if (front_visit.m_vertex_id == sink_id)
			{
			    bool non_conflicting_sink = true;

			    for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
			    {
				if (Conflicts[level].find(sink_id) != Conflicts[level].end())
				{
				    non_conflicting_sink = false;
				    break;
				}
			    }

			    if (non_conflicting_sink)
			    {
				sInt_32 prev_vertex_id = sink_id;
				Path.resize(front_visit.m_level + 1, -1);
/*
				printf("Path:\n");
				for (sInt_32 i = 0; i <= front_visit.m_level; ++i)
				{
				    for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
				    {
					printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
				    }
				    printf("\n");
				}
*/
				for (sInt_32 i = front_visit.m_level; i >= 0; --i)
				{
				    Path[i] = prev_vertex_id;
				    Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
				    
				    sASSERT(prev_visit != visited_Vertices[i].end());
				    
				    prev_vertex_id = prev_visit->second.m_previous_id;
				}
				return front_visit.m_level + 1;
			    }
			}
			
			Visit identity_visit(front_visit.m_level + 1, front_visit.m_vertex_id, front_visit.m_vertex_id);
			next_visited_Vertices[front_visit.m_vertex_id] = identity_visit;
			visit_Queue.push_back(identity_visit);
		    }
		}
	    }
	    visit_Queue.pop_front();
	}
	return -1;
    }


    sInt_32 sCBS::find_NonconflictingSequence(const sUndirectedGraph     &graph,
					      sInt_32                     source_id,
					      sInt_32                     sink_id,
					      const Conflicts_vector     &Conflicts,
					      const EdgeConflicts_vector &edge_Conflicts,					      
					      VertexIDs_vector           &Path) const
    {
	#ifdef sDEBUG
	{
	    printf("seq:%d -> %d\n", source_id, sink_id);
	}
	#endif

	Visits_list visit_Queue;
	Visits_vector visited_Vertices;
       
	#ifdef sDEBUG
	{
	    printf("conflicts:\n");
	    
	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (VertexIDs_uset::const_iterator conf = Conflicts[level].begin(); conf != Conflicts[level].end(); ++conf)
		{
		    printf("%d ", *conf);
		}
		printf("\n");
	    }

	    printf("edge conflicts:\n");
	    
	    for (sInt_32 level = 0; level < edge_Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (NeighborIDs_umap::const_iterator neigh = edge_Conflicts[level].begin(); neigh != edge_Conflicts[level].end(); ++neigh)
		{
		    printf("%d [", neigh->first);
		    for (VertexIDs_uset::const_iterator vertex = neigh->second.begin(); vertex != neigh->second.end(); ++vertex)
		    {
			printf("%d ", *vertex);
		    }
		    printf("] ");
		}
		printf("\n");
	    }	    
	}
	#endif

	if (source_id == sink_id)
	{
	    bool non_conflicting_sink = true;

	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		if (Conflicts[level].find(sink_id) != Conflicts[level].end())
		{
		    non_conflicting_sink = false;
		    break;
		}
	    }

	    if (non_conflicting_sink)
	    {
		Path.push_back(sink_id);
		return 1;
	    }
	}
  
	if (Conflicts.empty() || Conflicts[0].find(source_id) == Conflicts[0].end())
	{
	    visit_Queue.push_back(Visit(0, source_id, -1));
	    visited_Vertices.push_back(Visits_umap());
	    
	    Visit source_visit(0, source_id, -1);
	    visited_Vertices[0][source_id] = source_visit;
	}
	else
	{
	    return -1;
	}
	
	while (!visit_Queue.empty())
	{
	    const Visit &front_visit = visit_Queue.front();
            /*
	    #ifdef sDEBUG
	    {
		printf("  v:%d,%d,%d\n", front_visit.m_level, front_visit.m_vertex_id, front_visit.m_previous_id);
	    }
	    #endif
	    */

	    if (visited_Vertices.size() <= front_visit.m_level + 1)
	    {
		visited_Vertices.push_back(Visits_umap());
	    }
	    for (sVertex::Neighbors_list::const_iterator neighbor = graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.begin(); neighbor != graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (visited_Vertices.size() <= front_visit.m_level + 1)
		{
		    sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
		    visited_Vertices.push_back(Visits_umap());
		}
		Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];

		if (next_visited_Vertices.find(neighbor_id) == next_visited_Vertices.end())
		{
		    /*
		    #ifdef sDEBUG
		    {
			printf("    n:%d\n", neighbor_id);
		    }
		    #endif
		    */
		    
		    if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(neighbor_id) == Conflicts[front_visit.m_level + 1].end())
		    {
			NeighborIDs_umap::const_iterator edge_vertex;
			
			if (   edge_Conflicts.size() <= front_visit.m_level
			    || (edge_vertex = edge_Conflicts[front_visit.m_level].find(front_visit.m_vertex_id)) == edge_Conflicts[front_visit.m_level].end()
			    || edge_vertex->second.find(neighbor_id) == edge_vertex->second.end())
			{
			    /*
                            #ifdef sDEBUG
			    {
			        printf("      *\n");
			    }
                            #endif
			    */
			
			    Visit neighbor_visit(front_visit.m_level + 1, neighbor_id, front_visit.m_vertex_id);
			    next_visited_Vertices[neighbor_id] = neighbor_visit;

			    if (neighbor_id == sink_id)
			    {
				bool non_conflicting_sink = true;
				
				for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
				{
				    if (Conflicts[level].find(sink_id) != Conflicts[level].end())
				    {
					non_conflicting_sink = false;
					break;
				    }
				}
				
				if (non_conflicting_sink)
				{
				    sInt_32 prev_vertex_id = sink_id;
				    Path.resize(front_visit.m_level + 2, -1);
/* Lamda				    
				    #ifdef sDEBUG
				    {
					printf("Path:\n");
					for (sInt_32 i = 0; i <= front_visit.m_level + 1; ++i)
					{
					    for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
					    {
						printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
					    }
					    printf("\n");
					}
				    }
				    #endif

                                    #ifdef sDEBUG
				    {
					printf("P:");
				    }
				    #endif
*/
				    for (sInt_32 i = front_visit.m_level + 1; i >= 0; --i)
				    {
					Path[i] = prev_vertex_id;
					/*
                                        #ifdef sDEBUG
					{
					    printf("%d ", Path[i]);
					}
				        #endif
					*/
					Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
					
					sASSERT(prev_visit != visited_Vertices[i].end());
					
					prev_vertex_id = prev_visit->second.m_previous_id;
				    }
				    /*
				    #ifdef sDEBUG
				    {
					printf("\n");
				    }
				    #endif
				    */

				    return front_visit.m_level + 2;
				}			
			    }			    
			    visit_Queue.push_back(neighbor_visit);
			}
		    }
		}
	    }
	    { /* wait action */

		/*
                #ifdef sDEBUG
		{
		    printf("    -:%d\n", front_visit.m_vertex_id);
		}
                #endif		
		*/
		
		if (visited_Vertices.size() <= front_visit.m_level + 1)
		{
		    sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
		    visited_Vertices.push_back(Visits_umap());
		}		
		Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];

		if (next_visited_Vertices.find(front_visit.m_vertex_id) == next_visited_Vertices.end())
		{
		    if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(front_visit.m_vertex_id) == Conflicts[front_visit.m_level + 1].end())
		    {
			NeighborIDs_umap::const_iterator edge_vertex;
			
			if (   edge_Conflicts.size() <= front_visit.m_level
			    || (edge_vertex = edge_Conflicts[front_visit.m_level].find(front_visit.m_vertex_id)) == edge_Conflicts[front_visit.m_level].end()
			    || edge_vertex->second.find(front_visit.m_vertex_id) == edge_vertex->second.end())
			{
			
			/*
                        #ifdef sDEBUG
			{
			    printf("      *\n");
			}
                        #endif
			*/		
			    if (front_visit.m_vertex_id == sink_id)
			    {
				bool non_conflicting_sink = true;
				
				for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
				{
				    if (Conflicts[level].find(sink_id) != Conflicts[level].end())
				    {
					non_conflicting_sink = false;
					break;
				    }
				}
				
				if (non_conflicting_sink)
				{
				    sInt_32 prev_vertex_id = sink_id;
				    Path.resize(front_visit.m_level + 1, -1);
				    
				    /*
				    #ifdef sDEBUG
				    {
					printf("Path:\n");
					for (sInt_32 i = 0; i <= front_visit.m_level; ++i)
					{
					    for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
					    {
						printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
					    }
					    printf("\n");
					}
				    }
				    #endif
				    */
				    for (sInt_32 i = front_visit.m_level; i >= 0; --i)
				    {
					Path[i] = prev_vertex_id;
					Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
					
					sASSERT(prev_visit != visited_Vertices[i].end());
					
					prev_vertex_id = prev_visit->second.m_previous_id;
				    }
				    return front_visit.m_level + 1;
				}
			    }
			    
			    Visit identity_visit(front_visit.m_level + 1, front_visit.m_vertex_id, front_visit.m_vertex_id);
			    next_visited_Vertices[front_visit.m_vertex_id] = identity_visit;
			    visit_Queue.push_back(identity_visit);
			}
		    }
		}
	    }
	    visit_Queue.pop_front();
	}
	return -1;
    }


    sInt_32 sCBS::findStar_NonconflictingSequence(const sUndirectedGraph     &graph,
						  sInt_32                     source_id,
						  sInt_32                     sink_id,
						  sInt_32                     sUNUSED(cost_limit),
						  sInt_32                     extra_cost,
						  const Conflicts_vector     &Conflicts,
						  const EdgeConflicts_vector &edge_Conflicts,
						  VertexIDs_vector           &Path) const
    {
	#ifdef sDEBUG
	{
	    /*
	    printf("seq:%d -> %d\n", source_id, sink_id);
	    */
	}
	#endif
		
	Visits_list visit_Queue;
	Visits_vector visited_Vertices;
       
	#ifdef sDEBUG
	{
	    /*
	    printf("conflicts:\n");
	    
	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (VertexIDs_uset::const_iterator conf = Conflicts[level].begin(); conf != Conflicts[level].end(); ++conf)
		{
		    printf("%d ", *conf);
		}
		printf("\n");
	    }

	    printf("edge conflicts:\n");
	    
	    for (sInt_32 level = 0; level < edge_Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (NeighborIDs_umap::const_iterator neigh = edge_Conflicts[level].begin(); neigh != edge_Conflicts[level].end(); ++neigh)
		{
		    printf("%d [", neigh->first);
		    for (VertexIDs_uset::const_iterator vertex = neigh->second.begin(); vertex != neigh->second.end(); ++vertex)
		    {
			printf("%d ", *vertex);
		    }
		    printf("] ");
		}
		printf("\n");
	    }	    
	    */
	}
	#endif

	if (source_id == sink_id)
	{
	    bool non_conflicting_sink = true;

	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		if (Conflicts[level].find(sink_id) != Conflicts[level].end())
		{
		    non_conflicting_sink = false;
		    break;
		}
	    }

	    if (non_conflicting_sink)
	    {
		Path.push_back(sink_id);
		return 1;
	    }
	}
  
	if (Conflicts.empty() || Conflicts[0].find(source_id) == Conflicts[0].end())
	{
	    visit_Queue.push_back(Visit(0, source_id, -1));
	    visited_Vertices.push_back(Visits_umap());
	    
	    Visit source_visit(0, source_id, -1);
	    visited_Vertices[0][source_id] = source_visit;
	}
	else
	{
	    return -1;
	}
	
	while (!visit_Queue.empty())
	{
	    const Visit &front_visit = visit_Queue.front();
            /*
	    #ifdef sDEBUG
	    {
		printf("  v:%d,%d,%d\n", front_visit.m_level, front_visit.m_vertex_id, front_visit.m_previous_id);
	    }
	    #endif
	    */

	    if (visited_Vertices.size() <= front_visit.m_level + 1)
	    {
		visited_Vertices.push_back(Visits_umap());
	    }

	    if (front_visit.m_level + m_goal_Distances[sink_id][front_visit.m_vertex_id] <= m_goal_Distances[sink_id][source_id] + extra_cost)
	    {
		for (sVertex::Neighbors_list::const_iterator neighbor = graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.begin(); neighbor != graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		    if (visited_Vertices.size() <= front_visit.m_level + 1)
		    {
			sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
			visited_Vertices.push_back(Visits_umap());
		    }
		    Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];
		    
		    if (next_visited_Vertices.find(neighbor_id) == next_visited_Vertices.end())
		    {
		    /*
		    #ifdef sDEBUG
		    {
			printf("    n:%d\n", neighbor_id);
		    }
		    #endif
		    */
		    
			if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(neighbor_id) == Conflicts[front_visit.m_level + 1].end())
			{
			    NeighborIDs_umap::const_iterator edge_vertex;
			    
			    if (   edge_Conflicts.size() <= front_visit.m_level
				|| (edge_vertex = edge_Conflicts[front_visit.m_level].find(front_visit.m_vertex_id)) == edge_Conflicts[front_visit.m_level].end()
				|| edge_vertex->second.find(neighbor_id) == edge_vertex->second.end())
			    {
			    /*
                            #ifdef sDEBUG
			    {
			        printf("      *\n");
			    }
                            #endif
			    */
			
				Visit neighbor_visit(front_visit.m_level + 1, neighbor_id, front_visit.m_vertex_id);
				next_visited_Vertices[neighbor_id] = neighbor_visit;

				if (neighbor_id == sink_id)
				{
				    bool non_conflicting_sink = true;
				    
				    for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
				    {
					if (Conflicts[level].find(sink_id) != Conflicts[level].end())
					{
					    non_conflicting_sink = false;
					    break;
					}
				    }
				    
				    if (non_conflicting_sink)
				    {
					sInt_32 prev_vertex_id = sink_id;
					Path.resize(front_visit.m_level + 2, -1);

					/* Lambda
                                        #ifdef sDEBUG
					{
					    printf("Path:\n");
					    for (sInt_32 i = 0; i <= front_visit.m_level + 1; ++i)
					    {
						for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
						{
						    printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
						}
						printf("\n");
					    }
					}
				        #endif

                                        #ifdef sDEBUG
					{
					    printf("P:");
					}
				        #endif
					*/
					for (sInt_32 i = front_visit.m_level + 1; i >= 0; --i)
					{
					    Path[i] = prev_vertex_id;
					    /*
                                            #ifdef sDEBUG
					    {
						printf("%d ", Path[i]);
					    }
				            #endif
					    */
					    Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
					    
					    sASSERT(prev_visit != visited_Vertices[i].end());
					    
					    prev_vertex_id = prev_visit->second.m_previous_id;
					}
					/*
				        #ifdef sDEBUG
					{
					    printf("\n");
					}
				        #endif
					*/
					return front_visit.m_level + 2;
				    }	 		
				}			    
				visit_Queue.push_back(neighbor_visit);
			    }
			}
		    }
		}
	        { /* wait action */

		/*
                #ifdef sDEBUG
		{
		    printf("    -:%d\n", front_visit.m_vertex_id);
		}
                #endif		
		*/
		    
		    if (visited_Vertices.size() <= front_visit.m_level + 1)
		    {
			sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
			visited_Vertices.push_back(Visits_umap());
		    }		
		    Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];
		    
		    if (next_visited_Vertices.find(front_visit.m_vertex_id) == next_visited_Vertices.end())
		    {
			if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(front_visit.m_vertex_id) == Conflicts[front_visit.m_level + 1].end())
			{
			    NeighborIDs_umap::const_iterator edge_vertex;
			    
			    if (   edge_Conflicts.size() <= front_visit.m_level
				|| (edge_vertex = edge_Conflicts[front_visit.m_level].find(front_visit.m_vertex_id)) == edge_Conflicts[front_visit.m_level].end()
				|| edge_vertex->second.find(front_visit.m_vertex_id) == edge_vertex->second.end())
			    {
			
			    /*
                            #ifdef sDEBUG
			    {
			        printf("      *\n");
			    }
                            #endif
			    */		
				if (front_visit.m_vertex_id == sink_id)
				{
				    bool non_conflicting_sink = true;
				    
				    for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
				    {
					if (Conflicts[level].find(sink_id) != Conflicts[level].end())
					{
					    non_conflicting_sink = false;
					    break;
					}
				    }
				    
				    if (non_conflicting_sink)
				    {
					sInt_32 prev_vertex_id = sink_id;
					Path.resize(front_visit.m_level + 1, -1);
				
                                        #ifdef sDEBUG
					{
					    printf("Path:\n");
					    for (sInt_32 i = 0; i <= front_visit.m_level; ++i)
					    {
						for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
						{
						    printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
						}
						printf("\n");
					    }
					}
				        #endif
					
					for (sInt_32 i = front_visit.m_level; i >= 0; --i)
					{
					    Path[i] = prev_vertex_id;
					    Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
					    
					    sASSERT(prev_visit != visited_Vertices[i].end());
					    
					    prev_vertex_id = prev_visit->second.m_previous_id;
					}
					return front_visit.m_level + 1;
				    }
				}
				
				Visit identity_visit(front_visit.m_level + 1, front_visit.m_vertex_id, front_visit.m_vertex_id);
				next_visited_Vertices[front_visit.m_vertex_id] = identity_visit;
				visit_Queue.push_back(identity_visit);
			    }
			}
		    }
		}
	    }
	    visit_Queue.pop_front();
	}
	return -1;
    }


    sInt_32 sCBS::findSuperStar_NonconflictingSequence(const sUndirectedGraph     &graph,
						       sInt_32                     source_id,
						       sInt_32                     sink_id,
						       sInt_32                     sUNUSED(cost_limit),
						       sInt_32                     extra_cost,
						       const Conflicts_vector     &Conflicts,
						       const EdgeConflicts_vector &edge_Conflicts,
						       VertexIDs_vector           &Path) const
    {
	sDouble relaxation = 1.0;
	
	#ifdef sDEBUG
	{
	    /*
	    printf("seq:%d -> %d\n", source_id, sink_id);
	    */
	}
	#endif
		
	Visits_mmap visit_Queue;
	Visits_vector visited_Vertices;
       
	#ifdef sDEBUG
	{
	    /*
	    printf("conflicts:\n");
	    
	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (VertexIDs_uset::const_iterator conf = Conflicts[level].begin(); conf != Conflicts[level].end(); ++conf)
		{
		    printf("%d ", *conf);
		}
		printf("\n");
	    }

	    printf("edge conflicts:\n");
	    
	    for (sInt_32 level = 0; level < edge_Conflicts.size(); ++level)
	    {
		printf("%d: ", level);
		for (NeighborIDs_umap::const_iterator neigh = edge_Conflicts[level].begin(); neigh != edge_Conflicts[level].end(); ++neigh)
		{
		    printf("%d [", neigh->first);
		    for (VertexIDs_uset::const_iterator vertex = neigh->second.begin(); vertex != neigh->second.end(); ++vertex)
		    {
			printf("%d ", *vertex);
		    }
		    printf("] ");
		}
		printf("\n");
	    }	    
	    */
	}
	#endif

	if (source_id == sink_id)
	{
	    bool non_conflicting_sink = true;

	    for (sInt_32 level = 0; level < Conflicts.size(); ++level)
	    {
		if (Conflicts[level].find(sink_id) != Conflicts[level].end())
		{
		    non_conflicting_sink = false;
		    break;
		}
	    }

	    if (non_conflicting_sink)
	    {
		Path.push_back(sink_id);
		return 1;
	    }
	}
  
	if (Conflicts.empty() || Conflicts[0].find(source_id) == Conflicts[0].end())
	{
	    visit_Queue.insert(Visits_mmap::value_type(0 + relaxation * m_goal_Distances[sink_id][source_id], Visit(0, source_id, -1)));
	    visited_Vertices.push_back(Visits_umap());
	    
	    Visit source_visit(0, source_id, -1);
	    visited_Vertices[0][source_id] = source_visit;
	}
	else
	{
	    return -1;
	}
	
	while (!visit_Queue.empty())
	{
	    const Visit &front_visit = visit_Queue.begin()->second;    
	    Visits_mmap::iterator visit_erase = visit_Queue.begin();
	        
            /*
	    #ifdef sDEBUG
	    {
		printf("  v:%d,%d,%d\n", front_visit.m_level, front_visit.m_vertex_id, front_visit.m_previous_id);
	    }
	    #endif
	    */

	    if (visited_Vertices.size() <= front_visit.m_level + 1)
	    {
		visited_Vertices.push_back(Visits_umap());
	    }
    
	    if (front_visit.m_level + m_goal_Distances[sink_id][front_visit.m_vertex_id] <= relaxation * (m_goal_Distances[sink_id][source_id] + extra_cost))
	    {
		for (sVertex::Neighbors_list::const_iterator neighbor = graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.begin(); neighbor != graph.m_Vertices[front_visit.m_vertex_id].m_Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		    if (visited_Vertices.size() <= front_visit.m_level + 1)
		    {
			sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
			visited_Vertices.push_back(Visits_umap());
		    }
		    Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];
		    
		    if (next_visited_Vertices.find(neighbor_id) == next_visited_Vertices.end())
		    {
		    /*
		    #ifdef sDEBUG
		    {
			printf("    n:%d\n", neighbor_id);
		    }
		    #endif
		    */		    
			if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(neighbor_id) == Conflicts[front_visit.m_level + 1].end())
			{
			    NeighborIDs_umap::const_iterator edge_vertex;
			    
			    if (   edge_Conflicts.size() <= front_visit.m_level
				|| (edge_vertex = edge_Conflicts[front_visit.m_level].find(front_visit.m_vertex_id)) == edge_Conflicts[front_visit.m_level].end()
				|| edge_vertex->second.find(neighbor_id) == edge_vertex->second.end())
			    {
			    /*
                            #ifdef sDEBUG
			    {
			        printf("      *\n");
			    }
                            #endif
			    */			
				Visit neighbor_visit(front_visit.m_level + 1, neighbor_id, front_visit.m_vertex_id);
				next_visited_Vertices[neighbor_id] = neighbor_visit;

				if (neighbor_id == sink_id)
				{
				    bool non_conflicting_sink = true;
				    
				    for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
				    {
					if (Conflicts[level].find(sink_id) != Conflicts[level].end())
					{
					    non_conflicting_sink = false;
					    break;
					}
				    }
				    
				    if (non_conflicting_sink)
				    {
					sInt_32 prev_vertex_id = sink_id;
					Path.resize(front_visit.m_level + 2, -1);

					/* Lambda
                                        #ifdef sDEBUG
					{
					    printf("Path:\n");
					    for (sInt_32 i = 0; i <= front_visit.m_level + 1; ++i)
					    {
						for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
						{
						    printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
						}
						printf("\n");
					    }
					}
				        #endif

                                        #ifdef sDEBUG
					{
					    printf("P:");
					}
				        #endif
					*/
					for (sInt_32 i = front_visit.m_level + 1; i >= 0; --i)
					{
					    Path[i] = prev_vertex_id;
					    /*
                                            #ifdef sDEBUG
					    {
						printf("%d ", Path[i]);
					    }
				            #endif
					    */
					    Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
					    
					    sASSERT(prev_visit != visited_Vertices[i].end());
					    
					    prev_vertex_id = prev_visit->second.m_previous_id;
					}
					/*
				        #ifdef sDEBUG
					{
					    printf("\n");
					}
				        #endif
					*/
					return front_visit.m_level + 2;
				    }	 		
				}			    
				visit_Queue.insert(Visits_mmap::value_type(relaxation * m_goal_Distances[sink_id][neighbor_visit.m_vertex_id] + neighbor_visit.m_level, neighbor_visit));
			    }
			}
		    }
		}
	        { /* wait action */

		/*
                #ifdef sDEBUG
		{
		    printf("    -:%d\n", front_visit.m_vertex_id);
		}
                #endif		
		*/
		    
		    if (visited_Vertices.size() <= front_visit.m_level + 1)
		    {
			sASSERT(visited_Vertices.size() == front_visit.m_level + 1);
			visited_Vertices.push_back(Visits_umap());
		    }		
		    Visits_umap &next_visited_Vertices = visited_Vertices[front_visit.m_level + 1];
		    
		    if (next_visited_Vertices.find(front_visit.m_vertex_id) == next_visited_Vertices.end())
		    {
			if (Conflicts.size() <= front_visit.m_level + 1 || Conflicts[front_visit.m_level + 1].find(front_visit.m_vertex_id) == Conflicts[front_visit.m_level + 1].end())
			{
			    NeighborIDs_umap::const_iterator edge_vertex;
			    
			    if (   edge_Conflicts.size() <= front_visit.m_level
				|| (edge_vertex = edge_Conflicts[front_visit.m_level].find(front_visit.m_vertex_id)) == edge_Conflicts[front_visit.m_level].end()
				|| edge_vertex->second.find(front_visit.m_vertex_id) == edge_vertex->second.end())
			    {
			
			    /*
                            #ifdef sDEBUG
			    {
			        printf("      *\n");
			    }
                            #endif
			    */		
				if (front_visit.m_vertex_id == sink_id)
				{
				    bool non_conflicting_sink = true;
				    
				    for (sInt_32 level = front_visit.m_level + 1; level < Conflicts.size(); ++level)
				    {
					if (Conflicts[level].find(sink_id) != Conflicts[level].end())
					{
					    non_conflicting_sink = false;
					    break;
					}
				    }
				    
				    if (non_conflicting_sink)
				    {
					sInt_32 prev_vertex_id = sink_id;
					Path.resize(front_visit.m_level + 1, -1);
				
                                        #ifdef sDEBUG
					{
					    printf("Path:\n");
					    for (sInt_32 i = 0; i <= front_visit.m_level; ++i)
					    {
						for (Visits_umap::const_iterator visit_vertex = visited_Vertices[i].begin(); visit_vertex != visited_Vertices[i].end(); ++visit_vertex)
						{
						    printf("(%d,%d,%d) ", visit_vertex->second.m_level, visit_vertex->second.m_vertex_id, visit_vertex->second.m_previous_id);
						}
						printf("\n");
					    }
					}
				        #endif
					
					for (sInt_32 i = front_visit.m_level; i >= 0; --i)
					{
					    Path[i] = prev_vertex_id;
					    Visits_umap::const_iterator prev_visit = visited_Vertices[i].find(prev_vertex_id);
					    
					    sASSERT(prev_visit != visited_Vertices[i].end());
					    
					    prev_vertex_id = prev_visit->second.m_previous_id;
					}
					return front_visit.m_level + 1;
				    }
				}
				
				Visit identity_visit(front_visit.m_level + 1, front_visit.m_vertex_id, front_visit.m_vertex_id);
				next_visited_Vertices[front_visit.m_vertex_id] = identity_visit;
				visit_Queue.insert(Visits_mmap::value_type(relaxation * m_goal_Distances[sink_id][identity_visit.m_vertex_id] + identity_visit.m_level, identity_visit));
			    }
			}
		    }
		}
	    }
//	    visit_Queue.erase(visit_Queue.begin());
	    visit_Queue.erase(visit_erase);	    
	}
	return -1;
    }        


    void sCBS::equalize_NonconflictingSequences(AgentPaths_vector &agent_Paths) const
    {
	sInt_32 N_agents = agent_Paths.size();
	sInt_32 max_path_length = 0;
	
	for (sInt_32 agent_id = 1; agent_id < N_agents; ++agent_id)
	{
	    if (max_path_length < agent_Paths[agent_id].size())
	    {
		max_path_length = agent_Paths[agent_id].size();
	    }
	}
	for (sInt_32 agent_id = 1; agent_id < N_agents; ++agent_id)
	{
	    while (agent_Paths[agent_id].size() < max_path_length)
	    {
		agent_Paths[agent_id].push_back(agent_Paths[agent_id][agent_Paths[agent_id].size() - 1]);
	    }
	}	
    }


    void sCBS::equalize_NonconflictingSequences(const AgentPaths_vector &agent_Paths, AgentPaths_vector &equal_agent_Paths) const
    {
	equal_agent_Paths = agent_Paths;
	equalize_NonconflictingSequences(equal_agent_Paths);
    }


/*----------------------------------------------------------------------------*/

   void sCBS::rebuild_NodeConflictsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store)
    {
	if (m_delta_conflict_node_IDs[agent_id] < 0)
	{
	    rebuild_InitialNodeConflictsDelta(agent_id, node_id, search_Store);
	}
	else
	{
	    rebuild_SubsequentNodeConflictsDelta(agent_id, node_id, search_Store);
	}
    }


    void sCBS::rebuild_InitialNodeConflictsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store)
    {
	NodeIDs_vector collect_upward_IDs;

	for (sInt_32 nd_id = node_id; nd_id > 0; nd_id = search_Store[nd_id].m_upper_node_id)
	{
	    collect_upward_IDs.push_back(nd_id);
	}

	for (NodeIDs_vector::const_iterator collect_id = collect_upward_IDs.begin(); collect_id != collect_upward_IDs.end(); ++collect_id)
	{
	    if (search_Store[*collect_id].m_next_conflict != NULL && search_Store[*collect_id].m_next_conflict->m_agent_id == agent_id)
	    {
		sCBS_ADD_AGENT_DELTA_CONFLICT(agent_id,
					      search_Store[*collect_id].m_next_conflict->m_level,
					      search_Store[*collect_id].m_next_conflict->m_vertex_id);
	    }
	    if (search_Store[*collect_id].m_next_edge_conflict != NULL && search_Store[*collect_id].m_next_edge_conflict->m_agent_id == agent_id)
	    {
		sCBS_ADD_AGENT_DELTA_EDGE_CONFLICT(agent_id,
						   search_Store[*collect_id].m_next_edge_conflict->m_level,
						   search_Store[*collect_id].m_next_edge_conflict->m_edge_u_id,
						   search_Store[*collect_id].m_next_edge_conflict->m_edge_v_id);
	    }
	}	
	m_delta_conflict_node_IDs[agent_id] = node_id;
    }

    
    void sCBS::rebuild_SubsequentNodeConflictsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store)
    {
	NodeIDs_vector collect_upward_IDs, collect_downward_IDs;

	sInt_32 nd_id = node_id;
	sInt_32 dnd_id = m_delta_conflict_node_IDs[agent_id];

	while (nd_id != dnd_id)
	{
	    if (nd_id < dnd_id)
	    {
		collect_downward_IDs.push_back(dnd_id);
		dnd_id = search_Store[dnd_id].m_upper_node_id;
	    }
	    else
	    {
		collect_upward_IDs.push_back(nd_id);
		nd_id = search_Store[nd_id].m_upper_node_id;
	    }
	}

	for (NodeIDs_vector::const_iterator collect_id = collect_downward_IDs.begin(); collect_id != collect_downward_IDs.end(); ++collect_id)
	{
	    if (search_Store[*collect_id].m_next_conflict != NULL && search_Store[*collect_id].m_next_conflict->m_agent_id == agent_id)
	    {
		sCBS_DEL_AGENT_DELTA_CONFLICT(agent_id,
					      search_Store[*collect_id].m_next_conflict->m_level,
					      search_Store[*collect_id].m_next_conflict->m_vertex_id);
	    }
	    if (search_Store[*collect_id].m_next_edge_conflict != NULL && search_Store[*collect_id].m_next_edge_conflict->m_agent_id == agent_id)
	    {
		sCBS_DEL_AGENT_DELTA_EDGE_CONFLICT(agent_id,
						   search_Store[*collect_id].m_next_edge_conflict->m_level,
						   search_Store[*collect_id].m_next_edge_conflict->m_edge_u_id,
						   search_Store[*collect_id].m_next_edge_conflict->m_edge_v_id);
	    }	    	    
	}
	for (NodeIDs_vector::const_iterator collect_id = collect_upward_IDs.begin(); collect_id != collect_upward_IDs.end(); ++collect_id)
	{
	    if (search_Store[*collect_id].m_next_conflict != NULL && search_Store[*collect_id].m_next_conflict->m_agent_id == agent_id)
	    {
		sCBS_ADD_AGENT_DELTA_CONFLICT(agent_id,
					      search_Store[*collect_id].m_next_conflict->m_level,
					      search_Store[*collect_id].m_next_conflict->m_vertex_id);
	    }
	    if (search_Store[*collect_id].m_next_edge_conflict != NULL && search_Store[*collect_id].m_next_edge_conflict->m_agent_id == agent_id)
	    {
		sCBS_ADD_AGENT_DELTA_EDGE_CONFLICT(agent_id,
						   search_Store[*collect_id].m_next_edge_conflict->m_level,
						   search_Store[*collect_id].m_next_edge_conflict->m_edge_u_id,
						   search_Store[*collect_id].m_next_edge_conflict->m_edge_v_id);
	    }	    
	}
	m_delta_conflict_node_IDs[agent_id] = node_id;
    }


    void sCBS::rebuild_NodePathsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store)
    {
	if (m_delta_path_node_IDs[agent_id] < 0)
	{
	    rebuild_InitialNodePathsDelta(agent_id, node_id, search_Store);
	}
	else
	{
	    rebuild_SubsequentNodePathsDelta(agent_id, node_id, search_Store);
	}
    }


    void sCBS::rebuild_InitialNodePathsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store)
    {
	NodeIDs_vector collect_upward_IDs;

	for (sInt_32 nd_id = node_id; nd_id > 0; nd_id = search_Store[nd_id].m_upper_node_id)
	{
	    collect_upward_IDs.push_back(nd_id);
	}
	m_delta_agent_Paths = m_first_agent_Paths;

	for (NodeIDs_vector::const_reverse_iterator collect_id = collect_upward_IDs.rbegin(); collect_id != collect_upward_IDs.rend(); ++collect_id)
	{
	    if (search_Store[*collect_id].m_next_path != NULL && search_Store[*collect_id].m_upd_agent_id == agent_id)
	    {
		m_delta_agent_Paths[agent_id] = *search_Store[*collect_id].m_next_path;
	    }
	}	
	m_delta_path_node_IDs[agent_id] = node_id;
    }

    
    void sCBS::rebuild_SubsequentNodePathsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store)
    {
	NodeIDs_vector collect_upward_IDs, collect_downward_IDs;

	sInt_32 nd_id = node_id;
	sInt_32 dnd_id = m_delta_path_node_IDs[agent_id];

	while (nd_id != dnd_id)
	{
	    if (nd_id < dnd_id)
	    {
		collect_downward_IDs.push_back(dnd_id);
		dnd_id = search_Store[dnd_id].m_upper_node_id;
	    }
	    else
	    {
		collect_upward_IDs.push_back(nd_id);
		nd_id = search_Store[nd_id].m_upper_node_id;
	    }
	}
	collect_downward_IDs.push_back(dnd_id);
	collect_upward_IDs.push_back(nd_id);

	for (NodeIDs_vector::const_iterator collect_id = collect_downward_IDs.begin(); collect_id != collect_downward_IDs.end(); ++collect_id)
	{
	    if (search_Store[*collect_id].m_prev_path != NULL && search_Store[*collect_id].m_upd_agent_id == agent_id)
	    {
		m_delta_agent_Paths[agent_id] = *search_Store[*collect_id].m_prev_path;
	    }
	}
	
	for (NodeIDs_vector::const_reverse_iterator collect_id = collect_upward_IDs.rbegin(); collect_id != collect_upward_IDs.rend(); ++collect_id)
	{
	    if (search_Store[*collect_id].m_next_path != NULL && search_Store[*collect_id].m_upd_agent_id == agent_id)
	    {
		m_delta_agent_Paths[agent_id] = *search_Store[*collect_id].m_next_path;
	    }
	}
	
	m_delta_path_node_IDs[agent_id] = node_id;
    }

    
/*----------------------------------------------------------------------------*/

} // namespace boOX
