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
/* agent.cpp / 1-224_leibniz                                                  */
/*----------------------------------------------------------------------------*/
//
// Agent and multi-agent problem related structures.
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
#include "util/statistics.h"
#include "core/agent.h"

using namespace std;
using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sConfiguration

    sConfiguration::sConfiguration()
    {
	// nothing
    }
    

    sConfiguration::sConfiguration(sInt_32 N_Vertices, sInt_32 N_Agents, bool random)
	: m_agent_Locs(N_Agents + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    for (sInt_32 i = 0; i < N_Vertices; ++i)
	    {
		Vertices.push_back(i);
	    }

	    sInt_32 remain = N_Vertices;
	    for (sInt_32 r = N_Agents; r >= 1;)
	    {
		if (remain <= 0)
		{
		    break;
		}
		sInt_32 rnd = rand() % remain;
		place_Agent(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (sInt_32 i = 0; i <= N_Agents; ++i)
	    {
		m_agent_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sConfiguration::sConfiguration(const sConfiguration &start_configuration, sInt_32 N_Vertices, sInt_32 N_Agents, bool random)
	: m_agent_Locs(N_Agents + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    sInt_32 remain = 0;
	    for (sInt_32 i = 0; i < N_Vertices; ++i)
	    {
		if (start_configuration.get_VertexOccupancy(i) == 0)
		{
		    Vertices.push_back(i);
		    ++remain;
		}
	    }
	    for (sInt_32 r = N_Agents; r >= 1;)
	    {
		if (remain <= 0)
		{
		    break;
		}
		sInt_32 rnd = rand() % remain;
		place_Agent(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (sInt_32 i = 0; i <= N_Agents; ++i)
	    {
		m_agent_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sConfiguration::sConfiguration(const sConfiguration &start_configuration, sInt_32 N_Vertices, sInt_32 N_Agents, sInt_32 N_fixed, bool random)
	: m_agent_Locs(N_Agents + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    sInt_32 remain = 0;

	    for (sInt_32 i = 0; i < N_Vertices; ++i)
	    {
		if (start_configuration.get_VertexOccupancy(i) == 0)
		{
		    Vertices.push_back(i);
		    ++remain;
		}
	    }

	    sInt_32 r = N_Agents;
	    for (sInt_32 f = 0; f < N_fixed; ++f)
	    {
		place_Agent(r, start_configuration.get_AgentLocation(r));
		--r;
	    }
	    while (r >= 1)
	    {
		if (remain <= 0)
		{
		    break;
		}
		sInt_32 rnd = rand() % remain;
		place_Agent(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (sInt_32 i = 0; i <= N_Agents; ++i)
	    {
		m_agent_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }

    
/*----------------------------------------------------------------------------*/
    
    bool sConfiguration::operator==(const sConfiguration &agent_configuration) const
    {
	sASSERT(m_agent_Locs.size() == agent_configuration.m_agent_Locs.size());

	Agents_vector::const_iterator agent_A, agent_B;
	
	for (agent_A = m_agent_Locs.begin(), agent_B = agent_configuration.m_agent_Locs.begin();
	     agent_A != m_agent_Locs.end();
	     ++agent_A, ++agent_B)
	{
	    if (*agent_A != *agent_B)
	    {
		return false;
	    }
	}
	return true;
    }


    bool sConfiguration::operator<(const sConfiguration &agent_configuration) const
    {
	sASSERT(m_agent_Locs.size() == agent_configuration.m_agent_Locs.size());

	Agents_vector::const_iterator agent_A, agent_B;
	
	for (agent_A = m_agent_Locs.begin(), agent_B = agent_configuration.m_agent_Locs.begin();
	     agent_A != m_agent_Locs.end();
	     ++agent_A, ++agent_B)
	{
	    if (*agent_A < *agent_B)
	    {
		return true;
	    }
	    else
	    {
		if (*agent_A > *agent_B)
		{
		    return false;
		}
	    }
	}

	return false;
    }

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sConfiguration::get_AgentCount(void) const
    {
	return (m_agent_Locs.size() - 1);
    }

    
    sInt_32 sConfiguration::get_VertexCount(void) const
    {
	return m_vertex_Occups.size();
    }
	

    sInt_32 sConfiguration::get_AgentLocation(sInt_32 agent_id) const
    {
	sASSERT(agent_id > 0 && agent_id < m_agent_Locs.size());

	return m_agent_Locs[agent_id];
    }


    sInt_32 sConfiguration::get_VertexOccupancy(sInt_32 vertex_id) const
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	return m_vertex_Occups[vertex_id];
    }


    void sConfiguration::place_Agent(sInt_32 agent_id, sInt_32 vertex_id)
    {
	sASSERT(agent_id > 0 && agent_id < m_agent_Locs.size());
	sASSERT(vertex_id < m_vertex_Occups.size());
	sASSERT(m_vertex_Occups[vertex_id] == VACANT_VERTEX);
	
	m_agent_Locs[agent_id] = vertex_id;
	m_vertex_Occups[vertex_id] = agent_id;
    }


    void sConfiguration::remove_Agent(sInt_32 agent_id)
    {
	sASSERT(agent_id > 0 && agent_id < m_agent_Locs.size());

	m_vertex_Occups[m_agent_Locs[agent_id]] = VACANT_VERTEX;
	m_agent_Locs[agent_id] = UNDEFINED_LOCATION;
    }


    void sConfiguration::clean_Vertex(sInt_32 vertex_id)
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	m_agent_Locs[m_vertex_Occups[vertex_id]] = UNDEFINED_LOCATION;
	m_vertex_Occups[vertex_id] = VACANT_VERTEX;
    }


    void sConfiguration::move_Agent(sInt_32 agent_id, sInt_32 dest_vertex_id)
    {
	sASSERT(agent_id > 0 && agent_id < m_agent_Locs.size());
	sASSERT(dest_vertex_id < m_vertex_Occups.size());

//	printf("%d # %d -> %d\n", agent_id, m_agent_Locs[agent_id], dest_vertex_id);

	#ifdef sDEBUG
	{
	    if (m_vertex_Occups[dest_vertex_id] != VACANT_VERTEX)
	    {
		printf("--> %d # %d -> %d\n", agent_id, m_agent_Locs[agent_id], dest_vertex_id);
		printf("Target occupied by: %d\n", m_vertex_Occups[dest_vertex_id]);
	    }
	}
        #endif
	sASSERT(m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX);

	m_vertex_Occups[m_agent_Locs[agent_id]] = VACANT_VERTEX;
	m_agent_Locs[agent_id] = dest_vertex_id;
	m_vertex_Occups[dest_vertex_id] = agent_id;

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_move_Executions;
	}
	#endif
    }


    bool sConfiguration::verify_Move(sInt_32 agent_id, sInt_32 dest_vertex_id, const sUndirectedGraph &graph) const
    {
	if (   (agent_id > 0 && agent_id < m_agent_Locs.size())
	    && (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX)
	    && (m_vertex_Occups[m_agent_Locs[agent_id]] == agent_id)
	    && graph.is_Adjacent(m_agent_Locs[agent_id], dest_vertex_id))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d # %d -> %d (adjacent:%d)\n", agent_id, m_agent_Locs[agent_id], dest_vertex_id, graph.is_Adjacent(m_agent_Locs[agent_id], dest_vertex_id) ? 1 : 0);
	}
	#endif

	return false;
    }


    bool sConfiguration::verify_Move(sInt_32 agent_id, sInt_32 dest_vertex_id) const
    {
	if (   (agent_id > 0 && agent_id < m_agent_Locs.size())
	    && (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX)
	    && (m_vertex_Occups[m_agent_Locs[agent_id]] == agent_id))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d # %d -> %d\n", agent_id, m_agent_Locs[agent_id], dest_vertex_id);
	}
	#endif

	return false;
    }


    bool sConfiguration::check_Move(
#ifdef sDEBUG
	                               sInt_32 agent_id,
#else
	                               int,
#endif
				       sInt_32 dest_vertex_id) const
    {
	if (   (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d -> %d\n", agent_id, dest_vertex_id);
	}
	#endif

	return false;
    }


    void sConfiguration::generate_Walk(const sConfiguration &start_configuration, const sUndirectedGraph &environment)
    {
	for (sInt_32 agent_id = 1; agent_id < m_agent_Locs.size(); ++agent_id)
	{
	    const sVertex *vertex = environment.get_Vertex(start_configuration.get_AgentLocation(agent_id));
	    
	    for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }
	    for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Agent(agent_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
	    }
	}
    }


    void sConfiguration::generate_DisjointWalk(const sConfiguration &start_configuration, const sUndirectedGraph &environment)
    {
	for (sInt_32 agent_id = 1; agent_id < m_agent_Locs.size(); ++agent_id)
	{
	    const sVertex *vertex = environment.get_Vertex(start_configuration.get_AgentLocation(agent_id));
	    
	    for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }
	    bool stand_in = true;
	    
	    for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH || stand_in; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Agent(agent_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
		stand_in = false;
		
		if (start_configuration.get_AgentLocation(agent_id) == get_AgentLocation(agent_id))
		{
		    stand_in = true;
		}
	    }	    
	}
    }
    
    
    void sConfiguration::generate_NovelWalk(const sConfiguration &start_configuration, const sUndirectedGraph &environment)
    {
	sConfiguration agent_configuration = start_configuration;

	bool stand_in = false;
	
	for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH || stand_in; ++i)
	{	    
	    for (sInt_32 agent_id = 1; agent_id < m_agent_Locs.size(); ++agent_id)
	    {
		const sVertex *vertex = environment.get_Vertex(agent_configuration.get_AgentLocation(agent_id));

		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;

		    if (agent_configuration.get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		    {
			agent_configuration.move_Agent(agent_id, vertex->m_id);
		    }
		}
	    }
	    /*
	    stand_in = false;
	    for (sInt_32 agent_id = 1; agent_id < m_agent_Locs.size(); ++agent_id)
	    {
		if (start_configuration.get_AgentLocation(agent_id) == agent_configuration.get_AgentLocation(agent_id))
		{
		    stand_in = true;
		    break;
		}
	    }
	    */
	}
	*this = agent_configuration;
    }


    sResult sConfiguration::generate_Nonconflicting(sInt_32 N_Vertices, sInt_32 N_Agents, const sUndirectedGraph &environment)
    {
	m_agent_Locs.resize(N_Agents + 1);
	m_vertex_Occups.resize(N_Vertices);

	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	
	Vertices_vector Vertices;
	sInt_32 remain = 0;
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    Vertices.push_back(i);
	    ++remain;
	}	
	sInt_32 r = N_Agents;
	
	while (r >= 1)
	{
	    if (remain <= 0)
	    {
		return -1;
	    }
	    sInt_32 rnd = rand() % remain;
	    sInt_32 pos = Vertices[rnd];
	    place_Agent(r, Vertices[rnd]);
	    Vertices[rnd] = Vertices[--remain];

	    sVertex::VertexIDs_vector Conflicts = environment.m_Vertices[pos].m_Conflicts;
	    for (sVertex::VertexIDs_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
	    {
		for (sInt_32 i = 0; i < remain; ++i)
		{
		    if (Vertices[i] == *conflict)
		    {
			Vertices[i] = Vertices[--remain];
			break;
		    }
		}
	    }
	    --r;
	}
	
	return sRESULT_SUCCESS;
    }
    
    
    void sConfiguration::generate_NovelNonconflictingWalk(const sConfiguration &start_configuration, const sUndirectedGraph &environment)
    {
	sConfiguration agent_configuration = start_configuration;

	bool stand_in = false;
	
	for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH || stand_in; ++i)
	{	    
	    for (sInt_32 agent_id = 1; agent_id < m_agent_Locs.size(); ++agent_id)
	    {
		const sVertex *vertex = environment.get_Vertex(agent_configuration.get_AgentLocation(agent_id));

		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;

		    if (agent_configuration.get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		    {
			bool non_conflict = true;
			sVertex::VertexIDs_vector Conflicts = environment.m_Vertices[vertex->m_id].m_Conflicts;
			for (sVertex::VertexIDs_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
			{
			    if (agent_configuration.get_VertexOccupancy(*conflict) != VACANT_VERTEX && agent_configuration.get_VertexOccupancy(*conflict) != agent_id)
			    {
				non_conflict = false;
				break;
			    }
			}

			if (non_conflict)
			{			    
			    agent_configuration.move_Agent(agent_id, vertex->m_id);
			}
		    }
		}
	    }
	    /*
	    stand_in = false;
	    for (sInt_32 agent_id = 1; agent_id < m_agent_Locs.size(); ++agent_id)
	    {
		if (start_configuration.get_AgentLocation(agent_id) == agent_configuration.get_AgentLocation(agent_id))
		{
		    stand_in = true;
		    break;
		}
	    }
	    */
	}
	*this = agent_configuration;
    }        


    void sConfiguration::generate_Walk(const sConfiguration &start_configuration, const sUndirectedGraph &environment, sInt_32 N_fixed)
    {
	for (sInt_32 agent_id = 1; agent_id < N_fixed; ++agent_id)
	{
	    place_Agent(agent_id, start_configuration.get_AgentLocation(agent_id));
	}
	for (sInt_32 agent_id = N_fixed; agent_id < m_agent_Locs.size(); ++agent_id)
	{
	    const sVertex *vertex = environment.get_Vertex(start_configuration.get_AgentLocation(agent_id));

	    for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }

	    for (sInt_32 i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Agent(agent_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    sInt_32 rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (sInt_32 n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
	    }
	}
    }


  void sConfiguration::generate_Equidistant(const sConfiguration &start_configuration, sUndirectedGraph &environment, sInt_32 distance)
  {
    sInt_32 N_Agents = start_configuration.get_AgentCount();
    sInt_32 N_Vertices = environment.get_VertexCount();

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	  sInt_32 s_id = start_configuration.get_AgentLocation(agent_id);

	  VertexIDs_vector equidistant_IDs, free_equidistant_IDs;
	  environment.collect_EquidistantVertices(s_id, distance, equidistant_IDs);

	  for (VertexIDs_vector::const_iterator equidistant = equidistant_IDs.begin(); equidistant != equidistant_IDs.end(); ++equidistant)
	    {
	      if (get_VertexOccupancy(*equidistant) == VACANT_VERTEX)
		{
		  free_equidistant_IDs.push_back(*equidistant);
		}
	    }
	  if (free_equidistant_IDs.empty())
	    {
	      VertexIDs_vector free_vertex_IDs;

	      for (sInt_32 i = 0; i < N_Vertices; ++i)
		{
		  if (get_VertexOccupancy(i) == VACANT_VERTEX)
		    {
		      free_vertex_IDs.push_back(i);
		    }
		}
	      sInt_32 rnd = rand() % free_vertex_IDs.size();
	      place_Agent(agent_id, free_vertex_IDs[rnd]);
	    }
	  else
	    {
	      sInt_32 rnd = rand() % free_equidistant_IDs.size();
	      place_Agent(agent_id, free_equidistant_IDs[rnd]);
	    }
	}
  }


/*----------------------------------------------------------------------------*/

    void sConfiguration::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sConfiguration::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sAgent configuration: (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_agent_Locs.size() - 1, m_vertex_Occups.size());
	fprintf(fw, "%s%sagent locations: {", indent.c_str(), s_INDENT.c_str());
	
	sInt_32 N_Agents_1 = m_agent_Locs.size();
	for (sInt_32 i = 1; i < N_Agents_1; ++i)
	{
	    fprintf(fw, "%d#%d ", i, m_agent_Locs[i]);
	}
	fprintf(fw, "}\n");

	fprintf(fw, "%s%svertex occupancy: {", indent.c_str(), s_INDENT.c_str());
	
	sInt_32 N_Vertices = m_vertex_Occups.size();
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "%d#%d ", m_vertex_Occups[i], i);
	}
	fprintf(fw, "}\n");
/*
	if (!m_agent_Sizes.empty())
	{
	    fprintf(fw, "%s%s agent sizes: {", indent.c_str(), s_INDENT.c_str());
	    for (sInt_32 i = 1; i < N_Agents_1; ++i)
	    {
		fprintf(fw, "%d(%d) ", i, m_agent_Sizes[i]);
	    }
	    fprintf(fw, "}\n");
	}
*/
	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sConfiguration::to_Screen_brief(const sString &indent) const
    {
	to_Stream_brief(stdout, indent);
    }


    void sConfiguration::to_Stream_brief(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sAgent configuration (brief): (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_agent_Locs.size() - 1, m_vertex_Occups.size());
	fprintf(fw, "%s%s agent locations: {", indent.c_str(), s_INDENT.c_str());
	
	sInt_32 N_Agents_1 = m_agent_Locs.size();
	for (sInt_32 i = 1; i < N_Agents_1; ++i)
	{
	    fprintf(fw, "%d#%d ", i, m_agent_Locs[i]);
	}
	fprintf(fw, "}\n");

	fprintf(fw, "%s]\n", indent.c_str());
    }    


    sResult sConfiguration::to_File_cpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	to_Stream_cpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sConfiguration::to_File_ccpf(const sString &filename, const sUndirectedGraph &environment, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	to_Stream_ccpf(fw, environment, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sConfiguration::to_File_mpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	to_Stream_mpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sConfiguration::to_File_cmpf(const sString &filename, const sUndirectedGraph &environment, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	to_Stream_cmpf(fw, environment, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }        


    void sConfiguration::to_Stream_cpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_vertex_Occups.size();
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1)[%d:-1:-1]\n", i, m_vertex_Occups[i]);
	}
    }


    void sConfiguration::to_Stream_ccpf(FILE *fw, const sUndirectedGraph &environment, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_vertex_Occups.size();
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1:%d)[%d:-1:-1]\n", i, environment.m_Vertices[i].m_capacity, m_vertex_Occups[i]);
	}
    }    
    
    
    void sConfiguration::to_Stream_mpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_vertex_Occups.size();
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d,%d,-1)\n", i, m_vertex_Occups[i]);
	}
    }


    void sConfiguration::to_Stream_cmpf(FILE *fw, const sUndirectedGraph &environment, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_vertex_Occups.size();
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d,%d,%d,-1)\n", i, environment.m_Vertices[i].m_capacity, m_vertex_Occups[i]);
	}
    }    


    sResult sConfiguration::from_File_cpf(const sString &filename, sInt_32 component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	result = from_Stream_cpf(fr, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sConfiguration::from_File_ccpf(const sString &filename, sUndirectedGraph &environment, sInt_32 component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	result = from_Stream_ccpf(fr, environment, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }    


    sResult sConfiguration::from_File_mpf(const sString &filename, sInt_32 component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	result = from_Stream_mpf(fr, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sConfiguration::from_File_cmpf(const sString &filename, sUndirectedGraph &environment, sInt_32 component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_CONFIGURATION_OPEN_ERROR;
	}
	
	result = from_Stream_cmpf(fr, environment, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }        

    
    sResult sConfiguration::from_Stream_cpf(FILE *fr, sInt_32 component)
    {
	
	m_agent_Locs.clear();
	m_vertex_Occups.clear();

	sInt_32 N_Agents = 0;
	sInt_32 N_Vertices = 0;

	sInt_32 c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    sInt_32 vertex_id, cycle_id, agent_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[%d", &vertex_id, &cycle_id, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d:%d)[%d:%d", &vertex_id, &cycle_id, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		++N_Agents;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sAGENT_CONFIGURATION_SEEK_ERROR;
	}
	c = fgetc(fr);

	sInt_32 undefined_location = UNDEFINED_LOCATION;
	m_agent_Locs.resize(N_Agents + 1, undefined_location);	
	sInt_32 vacant_vertex = VACANT_VERTEX;
	m_vertex_Occups.resize(N_Vertices, vacant_vertex);

	while (c == '(')
	{
	    sInt_32 vertex_id, cycle_id, agent_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[%d", &vertex_id, &cycle_id, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d:%d)[%d:%d", &vertex_id, &cycle_id, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    case 2:
	    {
		sInt_32 dummy_agent_1_id, dummy_agent_2_id;
		fscanf(fr, "%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &dummy_agent_1_id, &dummy_agent_2_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		m_agent_Locs[agent_id] = vertex_id;
		m_vertex_Occups[vertex_id] = agent_id;
	    }
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;	
    }


    sResult sConfiguration::from_Stream_ccpf(FILE *fr, sUndirectedGraph &environment, sInt_32 component)
    {
	
	m_agent_Locs.clear();
	m_vertex_Occups.clear();

	sInt_32 N_Agents = 0;
	sInt_32 N_Vertices = 0;

	sInt_32 c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    sInt_32 vertex_id, cycle_id, agent_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d:%d)[%d", &vertex_id, &cycle_id, &capacity, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d:%d:%d)[%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		++N_Agents;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sAGENT_CONFIGURATION_SEEK_ERROR;
	}
	c = fgetc(fr);

	sInt_32 undefined_location = UNDEFINED_LOCATION;
	m_agent_Locs.resize(N_Agents + 1, undefined_location);
	sInt_32 vacant_vertex = VACANT_VERTEX;
	m_vertex_Occups.resize(N_Vertices, vacant_vertex);

	while (c == '(')
	{
	    sInt_32 vertex_id, cycle_id, agent_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d:%d)[%d", &vertex_id, &cycle_id, &capacity, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d:%d:%d)[%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    case 2:
	    {
		sInt_32 dummy_agent_1_id, dummy_agent_2_id;
		fscanf(fr, "%d:%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_agent_1_id, &dummy_agent_2_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		m_agent_Locs[agent_id] = vertex_id;
		m_vertex_Occups[vertex_id] = agent_id;
	    }
	    environment.m_Vertices[vertex_id].m_capacity = capacity;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;	
    }    


    sResult sConfiguration::from_Stream_mpf(FILE *fr, sInt_32 component)
    {
	m_agent_Locs.clear();
	m_vertex_Occups.clear();

	sInt_32 N_Agents = 0;
	sInt_32 N_Vertices = 0;

	sInt_32 c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    sInt_32 vertex_id, agent_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d,%d", &vertex_id, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d,%d,%d", &vertex_id, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		++N_Agents;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sAGENT_CONFIGURATION_SEEK_ERROR;
	}
	c = fgetc(fr);

	sInt_32 undefined_location = UNDEFINED_LOCATION;
	m_agent_Locs.resize(N_Agents + 1, undefined_location);
	sInt_32 vacant_vertex = VACANT_VERTEX;
	m_vertex_Occups.resize(N_Vertices, vacant_vertex);

	while (c == '(')
	{
	    sInt_32 vertex_id, agent_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d,%d", &vertex_id, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d,%d,%d", &vertex_id, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		m_agent_Locs[agent_id] = vertex_id;
		m_vertex_Occups[vertex_id] = agent_id;
	    }
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sConfiguration::from_Stream_cmpf(FILE *fr, sUndirectedGraph &environment, sInt_32 component)
    {
	m_agent_Locs.clear();
	m_vertex_Occups.clear();

	sInt_32 N_Agents = 0;
	sInt_32 N_Vertices = 0;

	sInt_32 c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    sInt_32 vertex_id, agent_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d,%d,%d", &vertex_id, &capacity, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d,%d,%d,%d", &vertex_id, &capacity, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		++N_Agents;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sAGENT_CONFIGURATION_SEEK_ERROR;
	}
	c = fgetc(fr);

	sInt_32 undefined_location = UNDEFINED_LOCATION;
	m_agent_Locs.resize(N_Agents + 1, undefined_location);
	sInt_32 vacant_vertex = VACANT_VERTEX;
	m_vertex_Occups.resize(N_Vertices, vacant_vertex);

	while (c == '(')
	{
	    sInt_32 vertex_id, agent_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d,%d,%d", &vertex_id, &capacity, &agent_id);
		break;
	    }
	    case 1:
	    {
		sInt_32 dummy_agent_1_id;
		fscanf(fr, "%d,%d,%d,%d", &vertex_id, &capacity, &dummy_agent_1_id, &agent_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (agent_id > 0)
	    {
		m_agent_Locs[agent_id] = vertex_id;
		m_vertex_Occups[vertex_id] = agent_id;
	    }
	    environment.m_Vertices[vertex_id].m_capacity = capacity;
	    
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }        

    
/*----------------------------------------------------------------------------*/
// sInstance

    sInstance::sInstance()
    {
	// nothing
    }


    sInstance::sInstance(const sUndirectedGraph &environment, const sConfiguration &start_configuration, const sConfiguration &goal_configuration)
	: m_environment(environment)
	, m_start_configuration(start_configuration)
	, m_goal_configuration(goal_configuration)
    {
	sASSERT(environment.get_VertexCount() == start_configuration.get_VertexCount() && environment.get_VertexCount() == goal_configuration.get_VertexCount());

	sUndirectedGraph::VertexPairs_vector vertex_Pairs;

	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent = 1; agent <= N_Agents; ++agent)
	{
	    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(start_configuration.get_AgentLocation(agent), goal_configuration.get_AgentLocation(agent)));
	}
    }


/*----------------------------------------------------------------------------*/


    void sInstance::collect_Endpoints(VertexIDs_vector &source_IDs, VertexIDs_vector &goal_IDs) const
    {
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    sInt_32 agent_sink_vertex_id;
	    
	    agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    source_IDs.push_back(agent_source_vertex_id);
	    goal_IDs.push_back(agent_sink_vertex_id);
	}
    }

    
    sInt_32 sInstance::estimate_TotalPathCost(sInt_32 &max_individual_cost)
    {	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	sInt_32 min_total_cost = 0;
	max_individual_cost = 0;

	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    sInt_32 agent_sink_vertex_id;
	    
	    agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];
	    min_total_cost += agent_cost;

	    if (agent_cost > max_individual_cost)
	    {
		max_individual_cost = agent_cost;
	    }
	}
	return min_total_cost;
    }


    sInt_32 sInstance::estimate_TotalSwappingCost(sInt_32 &max_individual_cost)
    {	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	sInt_32 min_total_cost = 0;
	max_individual_cost = 0;

	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    sInt_32 agent_sink_vertex_id;
	    
	    agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];
	    min_total_cost += agent_cost;

	    if (agent_cost > max_individual_cost)
	    {
		max_individual_cost = agent_cost;
	    }
	}
	return min_total_cost;
    }


    sInt_32 sInstance::estimate_TotalPermutationCost(sInt_32 &max_individual_cost)
    {	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	sInt_32 min_total_cost = 0;
	max_individual_cost = 0;

	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    sInt_32 agent_sink_vertex_id;
	    
	    agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];
	    min_total_cost += agent_cost;

	    if (agent_cost > max_individual_cost)
	    {
		max_individual_cost = agent_cost;
	    }
	}
	return min_total_cost;
    }


    sInt_32 sInstance::estimate_TotalRotationCost(sInt_32 &max_individual_cost)
    {	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	sInt_32 min_total_cost = 0;
	max_individual_cost = 0;

	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    sInt_32 agent_sink_vertex_id;
	    
	    agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];
	    min_total_cost += agent_cost;

	    if (agent_cost > max_individual_cost)
	    {
		max_individual_cost = agent_cost;
	    }
	}
	return min_total_cost;
    }    
    

    sInt_32 sInstance::construct_PathMDD(sInt_32     max_total_cost,
					 MDD_vector &MDD,
					 sInt_32    &extra_cost,
					 MDD_vector &extra_MDD)
    {
	return construct_GraphPathMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);	
    }

    
    sInt_32 sInstance::construct_GraphPathMDD(sUndirectedGraph &graph,
					      sInt_32           max_total_cost,
					      MDD_vector       &MDD,
					      sInt_32          &extra_cost,
					      MDD_vector       &extra_MDD)
    {
	sInt_32 max_individual_cost;
	sInt_32 N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	sInt_32 min_total_cost = estimate_TotalPathCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	sInt_32 mdd_depth = max_individual_cost + extra_cost;
	
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();

	MDD.resize(N_Agents + 1);
	extra_MDD.resize(N_Agents + 1);

	AgentIndices_mmap sorted_mdd_Agents;

	for (sInt_32 mdd_agent_id = 1; mdd_agent_id <= N_Agents; ++mdd_agent_id)
	{
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(mdd_agent_id);   
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    sorted_mdd_Agents.insert(AgentIndices_mmap::value_type(agent_cost, mdd_agent_id));
	}

	sInt_32 sort_index = 0;
	for (AgentIndices_mmap::const_reverse_iterator sort_agent = sorted_mdd_Agents.rbegin(); sort_agent != sorted_mdd_Agents.rend(); ++sort_agent)
	{
	    sInt_32 mdd_agent_id = sort_agent->second;
	    
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id  = m_goal_configuration.get_AgentLocation(mdd_agent_id);
   	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (sInt_32 mdd_level = source_Distances[agent_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(agent_cost + extra_cost - goal_Distances[agent_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{
		    MDD[mdd_agent_id][mdd_level].push_back(vertex_id);
		}
	    }

	    for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (MDD[mdd_agent_id][mdd_level].empty())
		{
		    MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
		if (   mdd_level >= source_Distances[agent_source_vertex_id][agent_sink_vertex_id]
		    && mdd_level < source_Distances[agent_source_vertex_id][agent_sink_vertex_id] + extra_cost)
		{
		    extra_MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
	    }
	    ++sort_index;
	}

        #ifdef sDEBUG
	{
	    /*
	    printf("<----\n");	
	    printf("MDD printout\n");
	    for (sInt_32 mdd_agent = 1; mdd_agent <= N_Agents; ++mdd_agent)
	    {	    
		printf("agent:%d\n", mdd_agent);
		for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
		{
		    for (sInt_32 i = 0; i < MDD[mdd_agent][mdd_level].size(); ++i)
		    {		    
			printf("%d ", MDD[mdd_agent][mdd_level][i]);
		    }
		    printf("\n");
		}
		printf("\n");
	    }
	    printf("<----\n");
	    */
	}
	#endif
	
	return mdd_depth;	
    }


    sInt_32 sInstance::construct_SwappingMDD(sInt_32     max_total_cost,
					     MDD_vector &MDD,
					     sInt_32    &extra_cost,
					     MDD_vector &extra_MDD)
    {
	return construct_GraphSwappingMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);	
    }

    
    sInt_32 sInstance::construct_GraphSwappingMDD(sUndirectedGraph &graph,
						  sInt_32           max_total_cost,
						  MDD_vector       &MDD,
						  sInt_32          &extra_cost,
						  MDD_vector       &extra_MDD)
    {
	sInt_32 max_individual_cost;
	sInt_32 N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	sInt_32 min_total_cost = estimate_TotalSwappingCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	sInt_32 mdd_depth = max_individual_cost + extra_cost;
	
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();

	MDD.resize(N_Agents + 1);
	extra_MDD.resize(N_Agents + 1);

	AgentIndices_mmap sorted_mdd_Agents;

	for (sInt_32 mdd_agent_id = 1; mdd_agent_id <= N_Agents; ++mdd_agent_id)
	{
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(mdd_agent_id);   
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    sorted_mdd_Agents.insert(AgentIndices_mmap::value_type(agent_cost, mdd_agent_id));
	}

	sInt_32 sort_index = 0;
	for (AgentIndices_mmap::const_reverse_iterator sort_agent = sorted_mdd_Agents.rbegin(); sort_agent != sorted_mdd_Agents.rend(); ++sort_agent)
	{
	    sInt_32 mdd_agent_id = sort_agent->second;
	    
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id  = m_goal_configuration.get_AgentLocation(mdd_agent_id);
   	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (sInt_32 mdd_level = source_Distances[agent_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(agent_cost + extra_cost - goal_Distances[agent_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{
		    MDD[mdd_agent_id][mdd_level].push_back(vertex_id);
		}
	    }

	    for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (MDD[mdd_agent_id][mdd_level].empty())
		{
		    MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
		if (   mdd_level >= source_Distances[agent_source_vertex_id][agent_sink_vertex_id]
		    && mdd_level < source_Distances[agent_source_vertex_id][agent_sink_vertex_id] + extra_cost)
		{
		    extra_MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
	    }
	    ++sort_index;
	}

        #ifdef sDEBUG
	{
	    printf("<----\n");	
	    printf("MDD printout\n");
	    for (sInt_32 mdd_agent = 1; mdd_agent <= N_Agents; ++mdd_agent)
	    {	    
		printf("agent:%d\n", mdd_agent);
		for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
		{
		    for (sInt_32 i = 0; i < MDD[mdd_agent][mdd_level].size(); ++i)
		    {		    
			printf("%d ", MDD[mdd_agent][mdd_level][i]);
		    }
		    printf("\n");
		}
		printf("\n");
	    }
	    printf("<----\n");
	}
	#endif
	
	return mdd_depth;	
    }


    sInt_32 sInstance::construct_PermutationMDD(sInt_32     max_total_cost,
						MDD_vector &MDD,
						sInt_32    &extra_cost,
						MDD_vector &extra_MDD)
    {
	return construct_GraphPermutationMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);	
    }

    
    sInt_32 sInstance::construct_GraphPermutationMDD(sUndirectedGraph &graph,
						     sInt_32           max_total_cost,
						     MDD_vector       &MDD,
						     sInt_32          &extra_cost,
						     MDD_vector       &extra_MDD)
    {
	sInt_32 max_individual_cost;
	sInt_32 N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	sInt_32 min_total_cost = estimate_TotalPermutationCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	sInt_32 mdd_depth = max_individual_cost + extra_cost;
	
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();

	MDD.resize(N_Agents + 1);
	extra_MDD.resize(N_Agents + 1);

	AgentIndices_mmap sorted_mdd_Agents;

	for (sInt_32 mdd_agent_id = 1; mdd_agent_id <= N_Agents; ++mdd_agent_id)
	{
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(mdd_agent_id);   
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    sorted_mdd_Agents.insert(AgentIndices_mmap::value_type(agent_cost, mdd_agent_id));
	}

	sInt_32 sort_index = 0;
	for (AgentIndices_mmap::const_reverse_iterator sort_agent = sorted_mdd_Agents.rbegin(); sort_agent != sorted_mdd_Agents.rend(); ++sort_agent)
	{
	    sInt_32 mdd_agent_id = sort_agent->second;
	    
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id  = m_goal_configuration.get_AgentLocation(mdd_agent_id);
   	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (sInt_32 mdd_level = source_Distances[agent_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(agent_cost + extra_cost - goal_Distances[agent_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{
		    MDD[mdd_agent_id][mdd_level].push_back(vertex_id);
		}
	    }

	    for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (MDD[mdd_agent_id][mdd_level].empty())
		{
		    MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
		if (   mdd_level >= source_Distances[agent_source_vertex_id][agent_sink_vertex_id]
		    && mdd_level < source_Distances[agent_source_vertex_id][agent_sink_vertex_id] + extra_cost)
		{
		    extra_MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
	    }
	    ++sort_index;
	}

        #ifdef sDEBUG
	{
	    printf("<----\n");	
	    printf("MDD printout\n");
	    for (sInt_32 mdd_agent = 1; mdd_agent <= N_Agents; ++mdd_agent)
	    {	    
		printf("agent:%d\n", mdd_agent);
		for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
		{
		    for (sInt_32 i = 0; i < MDD[mdd_agent][mdd_level].size(); ++i)
		    {		    
			printf("%d ", MDD[mdd_agent][mdd_level][i]);
		    }
		    printf("\n");
		}
		printf("\n");
	    }
	    printf("<----\n");
	}
	#endif
	
	return mdd_depth;	
    }        


    sInt_32 sInstance::construct_RotationMDD(sInt_32     max_total_cost,
					     MDD_vector &MDD,
					     sInt_32    &extra_cost,
					     MDD_vector &extra_MDD)
    {
	return construct_GraphRotationMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);	
    }

    
    sInt_32 sInstance::construct_GraphRotationMDD(sUndirectedGraph &graph,
						  sInt_32           max_total_cost,
						  MDD_vector       &MDD,
						  sInt_32          &extra_cost,
						  MDD_vector       &extra_MDD)
    {
	sInt_32 max_individual_cost;
	sInt_32 N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	sInt_32 min_total_cost = estimate_TotalRotationCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	sInt_32 mdd_depth = max_individual_cost + extra_cost;
	
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();

	MDD.resize(N_Agents + 1);
	extra_MDD.resize(N_Agents + 1);

	AgentIndices_mmap sorted_mdd_Agents;

	for (sInt_32 mdd_agent_id = 1; mdd_agent_id <= N_Agents; ++mdd_agent_id)
	{
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id = m_goal_configuration.get_AgentLocation(mdd_agent_id);   
	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    sorted_mdd_Agents.insert(AgentIndices_mmap::value_type(agent_cost, mdd_agent_id));
	}

	sInt_32 sort_index = 0;
	for (AgentIndices_mmap::const_reverse_iterator sort_agent = sorted_mdd_Agents.rbegin(); sort_agent != sorted_mdd_Agents.rend(); ++sort_agent)
	{
	    sInt_32 mdd_agent_id = sort_agent->second;
	    
	    MDD[mdd_agent_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_agent_id].resize(mdd_depth + 1);

	    sInt_32 agent_source_vertex_id = m_start_configuration.get_AgentLocation(mdd_agent_id);
	    sInt_32 agent_sink_vertex_id  = m_goal_configuration.get_AgentLocation(mdd_agent_id);
   	    sInt_32 agent_cost = source_Distances[agent_source_vertex_id][agent_sink_vertex_id];

	    for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (sInt_32 mdd_level = source_Distances[agent_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(agent_cost + extra_cost - goal_Distances[agent_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{
		    MDD[mdd_agent_id][mdd_level].push_back(vertex_id);
		}
	    }

	    for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (MDD[mdd_agent_id][mdd_level].empty())
		{
		    MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
		if (   mdd_level >= source_Distances[agent_source_vertex_id][agent_sink_vertex_id]
		    && mdd_level < source_Distances[agent_source_vertex_id][agent_sink_vertex_id] + extra_cost)
		{
		    extra_MDD[mdd_agent_id][mdd_level].push_back(agent_sink_vertex_id);
		}
	    }
	    ++sort_index;
	}

        #ifdef sDEBUG
	{
	    printf("<----\n");	
	    printf("MDD printout\n");
	    for (sInt_32 mdd_agent = 1; mdd_agent <= N_Agents; ++mdd_agent)
	    {	    
		printf("agent:%d\n", mdd_agent);
		for (sInt_32 mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
		{
		    for (sInt_32 i = 0; i < MDD[mdd_agent][mdd_level].size(); ++i)
		    {		    
			printf("%d ", MDD[mdd_agent][mdd_level][i]);
		    }
		    printf("\n");
		}
		printf("\n");
	    }
	    printf("<----\n");
	}
	#endif
	
	return mdd_depth;	
    }


    void sInstance::construct_InverseMDD(const MDD_vector &MDD, InverseMDD_vector &inverse_MDD) const
    {
	sInt_32 N_agents = MDD.size() - 1;
	inverse_MDD.resize(N_agents + 1);

	for (sInt_32 mdd_agent_id = 1; mdd_agent_id <= N_agents; ++mdd_agent_id)
	{
	    sInt_32 N_levels = MDD[mdd_agent_id].size();	   
	    inverse_MDD[mdd_agent_id].resize(N_levels);

	    for (sInt_32 mdd_level = 0; mdd_level < N_levels; ++mdd_level)
	    {
		sInt_32 N_vertices = MDD[mdd_agent_id][mdd_level].size();
		
		for (sInt_32 mdd_vertex = 0; mdd_vertex < N_vertices; ++mdd_vertex)
		{
		    inverse_MDD[mdd_agent_id][mdd_level].insert(InverseVertexIDs_umap::value_type(MDD[mdd_agent_id][mdd_level][mdd_vertex], mdd_vertex));
		}
	    }
	}    
    }

    
/*----------------------------------------------------------------------------*/

    void sInstance::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sInstance::to_Screen_cpf(const sString &indent) const
    {
	to_Stream_cpf(stdout, indent);
    }


    void sInstance::to_Screen_ccpf(const sString &indent) const
    {
	to_Stream_ccpf(stdout, indent);
    }    


    void sInstance::to_Screen_mpf(const sString &indent) const
    {
	to_Stream_mpf(stdout, indent);
    }

    
    void sInstance::to_Screen_cmpf(const sString &indent) const
    {
	to_Stream_cmpf(stdout, indent);
    }        


    void sInstance::to_Screen_domainPDDL(const sString &indent) const
    {
	to_Stream_domainPDDL(stdout, indent);
    }


    void sInstance::to_Screen_problemPDDL(const sString &indent) const
    {
	to_Stream_problemPDDL(stdout, indent);
    }


    void sInstance::to_Screen_bgu(const sString &indent, sInt_32 instance_id) const
    {
	to_Stream_bgu(stdout, indent, instance_id);
    }


    sResult sInstance::to_File(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_cpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream_cpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_cpf(const sString &filename, sInt_32 N_agents, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream_cpf(fw, N_agents, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sInstance::to_File_ccpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream_ccpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sInstance::to_File_mpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream_mpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_mpf(const sString &filename, sInt_32 N_agents, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream_mpf(fw, N_agents, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sInstance::to_File_cmpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_OPEN_ERROR;
	}
	to_Stream_cmpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }        


    sResult sInstance::to_File_domainPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_PDDL_OPEN_ERROR;
	}
	to_Stream_domainPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_problemPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_PDDL_OPEN_ERROR;
	}
	to_Stream_problemPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_bgu(const sString &filename, const sString &indent, sInt_32 instance_id) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_BGU_OPEN_ERROR;
	}
	to_Stream_bgu(fw, indent, instance_id);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_bgu(const sString &filename, sInt_32 N_agents, const sString &indent, sInt_32 instance_id) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_BGU_OPEN_ERROR;
	}
	to_Stream_bgu(fw, N_agents, indent, instance_id);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    void sInstance::to_Stream(FILE *fw, const sString &indent) const
    {       
	fprintf(fw, "%sMultiagent instance: [\n", indent.c_str());
	fprintf(fw, "%s%sEnvironment:\n", indent.c_str(), s_INDENT.c_str());
	m_environment.to_Stream_vertices(fw, indent + s_INDENT + s_INDENT);
	fprintf(fw, "%s%sStart configuration:\n", indent.c_str(), s_INDENT.c_str());
	m_start_configuration.to_Stream(fw, indent + s_INDENT + s_INDENT);

	fprintf(fw, "%s%sGoal configuration:\n", indent.c_str(), s_INDENT.c_str());
	m_goal_configuration.to_Stream(fw, indent + s_INDENT + s_INDENT);
	
	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sInstance::to_Stream_cpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_start_configuration.m_vertex_Occups.size();
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1)[%d:%d:%d]", i,
		    m_start_configuration.m_vertex_Occups[i],
		    m_goal_configuration.m_vertex_Occups[i],
		    m_goal_configuration.m_vertex_Occups[i]);
	    
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (sInt_32 c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_cpf(fw, indent);
    }


    void sInstance::to_Stream_cpf(FILE *fw, sInt_32 N_agents, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_start_configuration.m_vertex_Occups.size();
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1)[%d:%d:%d]", i,
		    m_start_configuration.m_vertex_Occups[i] <= N_agents ? m_start_configuration.m_vertex_Occups[i] : 0,
		    m_goal_configuration.m_vertex_Occups[i] <= N_agents ? m_goal_configuration.m_vertex_Occups[i] : 0,
		    m_goal_configuration.m_vertex_Occups[i] <= N_agents ? m_goal_configuration.m_vertex_Occups[i] : 0);
	    
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (sInt_32 c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_cpf(fw, indent);
    }    


    void sInstance::to_Stream_ccpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_start_configuration.m_vertex_Occups.size();
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1:%d)[%d:%d:%d]", i,
		    m_environment.m_Vertices[i].m_capacity,
		    m_start_configuration.m_vertex_Occups[i],
		    m_goal_configuration.m_vertex_Occups[i],
		    m_goal_configuration.m_vertex_Occups[i]);
	    
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (sInt_32 c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_ccpf(fw, indent);
    }    


    void sInstance::to_Stream_mpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_start_configuration.m_vertex_Occups.size();
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d,%d,%d)", i,
		    m_start_configuration.m_vertex_Occups[i],
		    m_goal_configuration.m_vertex_Occups[i]);
	    
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (sInt_32 c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_mpf(fw, indent);
    }


    void sInstance::to_Stream_mpf(FILE *fw, sInt_32 N_agents, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_start_configuration.m_vertex_Occups.size();
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d,%d,%d)", i,
		    m_start_configuration.m_vertex_Occups[i] <= N_agents ? m_start_configuration.m_vertex_Occups[i] : 0,
		    m_goal_configuration.m_vertex_Occups[i] <= N_agents ? m_goal_configuration.m_vertex_Occups[i] : 0);
	    
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (sInt_32 c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_mpf(fw, indent);
    }    


    void sInstance::to_Stream_cmpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	sInt_32 N_Vertices = m_start_configuration.m_vertex_Occups.size();
	
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d,%d,%d,%d)", i,
		    m_environment.m_Vertices[i].m_capacity,
		    m_start_configuration.m_vertex_Occups[i],
		    m_goal_configuration.m_vertex_Occups[i]);
	    
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (sInt_32 c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_mpf(fw, indent);
    }        


    void sInstance::to_Stream_domainPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (domain multiagent)\n", indent.c_str());
	fprintf(fw, "%s%s(:predicates\n", indent.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s(adjacent ?u ?v)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s(agent_location ?r ?v)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s(no_agent ?v)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s%s(:action move\n", indent.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s:parameters (?r ?u ?v)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s:precondition (and\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(agent_location ?r ?u) (no_agent ?v) (adjacent ?u ?v)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s:effect (and\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(agent_location ?r ?v) (no_agent ?u) (not (no_agent ?v))\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sInstance::to_Stream_problemPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (problem multiagent_instance)\n", indent.c_str());
	fprintf(fw, "%s%s(:domain multiagent)\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s%s(:objects\n", indent.c_str(), s_INDENT.c_str());

	sInt_32 N_Vertices = m_environment.get_VertexCount();
	for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    fprintf(fw, "%s%s%sv%d\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), vertex_id);
	}

	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%s%sr%d\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), agent_id);
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s%s(:init\n", indent.c_str(), s_INDENT.c_str());

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), edge->m_arc_vu.m_target->m_id, edge->m_arc_uv.m_target->m_id);
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), edge->m_arc_uv.m_target->m_id, edge->m_arc_vu.m_target->m_id);
	}

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%s%s(agent_location r%d v%d)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), agent_id, m_start_configuration.get_AgentLocation(agent_id));
	}
	for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sInt_32 agent_id = m_start_configuration.get_VertexOccupancy(vertex_id);

	    if (agent_id == sConfiguration::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_agent v%d)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), vertex_id);
	    }
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s%s(:goal (and \n", indent.c_str(), s_INDENT.c_str());
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%s%s(agent_location r%d v%d)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), agent_id, m_goal_configuration.get_AgentLocation(agent_id));
	}
	for (sInt_32 vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sInt_32 agent_id = m_goal_configuration.get_VertexOccupancy(vertex_id);
		
	    if (agent_id == sConfiguration::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_agent v%d)\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), vertex_id);
	    }
	}
	fprintf(fw, "%s%s))\n", indent.c_str(), s_INDENT.c_str());
	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sInstance::to_Stream_bgu(FILE *fw, const sString &indent, sInt_32 instance_id) const
    {
	sASSERT(m_environment.m_Matrix != NULL);

	fprintf(fw, "%s%d\n", indent.c_str(), instance_id);
	fprintf(fw, "%sGrid:\n", indent.c_str());
	fprintf(fw, "%s%d,%d\n", indent.c_str(), m_environment.m_y_size, m_environment.m_x_size);

	for (sInt_32 j = 0; j < m_environment.m_y_size; ++j)
	{
	    for (sInt_32 i = 0; i < m_environment.m_x_size; ++i)
	    {
		if (m_environment.m_Matrix[j * m_environment.m_x_size + i] >= 0)
		{
		    fprintf(fw, ".");
		}
		else
		{
		    fprintf(fw, "@");
		}
	    }
	    fprintf(fw, "\n");
	}
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();

	fprintf(fw, "%sAgents:\n", indent.c_str());
	fprintf(fw, "%s%d\n", indent.c_str(), N_Agents);

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%d,", indent.c_str(), agent_id - 1);
	    
	    sInt_32 goal_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    fprintf(fw, "%d,%d,",  m_environment.calc_GridRow(goal_vertex_id), m_environment.calc_GridColumn(goal_vertex_id));
	    
	    sInt_32 init_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    fprintf(fw, "%d,%d\n", m_environment.calc_GridRow(init_vertex_id), m_environment.calc_GridColumn(init_vertex_id));
	}
    }


    void sInstance::to_Stream_bgu(FILE *fw, sInt_32 N_agents, const sString &indent, sInt_32 instance_id) const
    {
	sASSERT(m_environment.m_Matrix != NULL);

	fprintf(fw, "%s%d\n", indent.c_str(), instance_id);
	fprintf(fw, "%sGrid:\n", indent.c_str());
	fprintf(fw, "%s%d,%d\n", indent.c_str(), m_environment.m_y_size, m_environment.m_x_size);

	for (sInt_32 j = 0; j < m_environment.m_y_size; ++j)
	{
	    for (sInt_32 i = 0; i < m_environment.m_x_size; ++i)
	    {
		if (m_environment.m_Matrix[j * m_environment.m_x_size + i] >= 0)
		{
		    fprintf(fw, ".");
		}
		else
		{
		    fprintf(fw, "@");
		}
	    }
	    fprintf(fw, "\n");
	}
	
	fprintf(fw, "%sAgents:\n", indent.c_str());
	fprintf(fw, "%s%d\n", indent.c_str(), N_agents);

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    fprintf(fw, "%s%d,", indent.c_str(), agent_id - 1);
	    
	    sInt_32 goal_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    fprintf(fw, "%d,%d,",  m_environment.calc_GridRow(goal_vertex_id), m_environment.calc_GridColumn(goal_vertex_id));
	    
	    sInt_32 init_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    fprintf(fw, "%d,%d\n", m_environment.calc_GridRow(init_vertex_id), m_environment.calc_GridColumn(init_vertex_id));
	}
    }    


    sResult sInstance::from_File_cpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_CPF_OPEN_ERROR;
	}
	
	result = from_Stream_cpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }


    sResult sInstance::from_File_ccpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_CCPF_OPEN_ERROR;
	}
	
	result = from_Stream_ccpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }    


    sResult sInstance::from_File_mpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_MPF_OPEN_ERROR;
	}
	
	result = from_Stream_mpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }


    sResult sInstance::from_File_cmpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_MPF_OPEN_ERROR;
	}
	
	result = from_Stream_cmpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }        

    
    sResult sInstance::from_Stream_cpf(FILE *fr)
    {
	sResult result;

	if (sFAILED(result = m_environment.from_Stream_cpf(fr)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_start_configuration.from_Stream_cpf(fr, 0)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_goal_configuration.from_Stream_cpf(fr, 2)))
	{
	    return result;
	}
	
	return sRESULT_SUCCESS;
    }


    sResult sInstance::from_Stream_ccpf(FILE *fr)
    {
	sResult result;

	if (sFAILED(result = m_environment.from_Stream_ccpf(fr)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_start_configuration.from_Stream_ccpf(fr, m_environment, 0)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_goal_configuration.from_Stream_ccpf(fr, m_environment, 2)))
	{
	    return result;
	}
	
	return sRESULT_SUCCESS;
    }    


    sResult sInstance::from_Stream_mpf(FILE *fr)
    {
	sResult result;

	if (sFAILED(result = m_environment.from_Stream_mpf(fr)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_start_configuration.from_Stream_mpf(fr, 0)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_goal_configuration.from_Stream_mpf(fr, 1)))
	{
	    return result;
	}
	
	return sRESULT_SUCCESS;
    }


    sResult sInstance::from_Stream_cmpf(FILE *fr)
    {
	sResult result;

	if (sFAILED(result = m_environment.from_Stream_cmpf(fr)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_start_configuration.from_Stream_cmpf(fr, m_environment, 0)))
	{
	    return result;
	}

	if (fseek(fr, 0, SEEK_SET) < 0)
	{
	    return sAGENT_INSTANCE_SEEK_ERROR;
	}
	if (sFAILED(result = m_goal_configuration.from_Stream_cmpf(fr, m_environment, 1)))
	{
	    return result;
	}
	
	return sRESULT_SUCCESS;
    }        


    sResult sInstance::from_File_bgu(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_BGU_OPEN_ERROR;
	}
	
	result = from_Stream_bgu(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::from_Stream_bgu(FILE *fr)
    {
	sResult result;
	sInt_32 N_Agents;
	
	if (sFAILED(result = m_environment.from_Stream_bgu(fr)))
	{
	    return result;
	}

	fscanf(fr, "Agents:\n");
	fscanf(fr, "%d\n", &N_Agents);

	m_start_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);
	m_goal_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 r_id;
	    fscanf(fr, "%d,", &r_id);
	    
	    sInt_32 goal_row, goal_column;
	    fscanf(fr, "%d,%d,", &goal_row, &goal_column);

	    sInt_32 goal_vertex_id = m_environment.calc_GridVertexID(goal_row, goal_column);
	    
	    sInt_32 init_row, init_column;
	    fscanf(fr, "%d,%d\n", &init_row, &init_column);
	    sInt_32 init_vertex_id =  m_environment.calc_GridVertexID(init_row, init_column);

	    m_start_configuration.place_Agent(agent_id, init_vertex_id);
	    m_goal_configuration.place_Agent(agent_id, goal_vertex_id);
	}

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_File_usc(const sString &map_filename, const sString &agents_filename) const
    {
	FILE *fw_map, *fw_agents;

	if ((fw_map = fopen(map_filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_USC_MAP_OPEN_ERROR;
	}
	if ((fw_agents = fopen(agents_filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_USC_AGNT_OPEN_ERROR;
	}
	
	to_Stream_usc(fw_map, fw_agents);
	
	fclose(fw_map);
	fclose(fw_agents);	

	return sRESULT_SUCCESS;	
    }

    
    void sInstance::to_Stream_usc(FILE *fw_map, FILE *fw_agents, const sString &indent) const
    {
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();
	sASSERT(N_Agents == m_goal_configuration.get_AgentCount());

	m_environment.to_Stream_usc(fw_map);
        fprintf(fw_agents, "%s%d\n", indent.c_str(), N_Agents);

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw_agents, "%s", indent.c_str());
	    
	    sInt_32 goal_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    sInt_32 goal_row = m_environment.calc_GridRow(goal_vertex_id);
	    sInt_32 goal_column = m_environment.calc_GridColumn(goal_vertex_id);
	    
	    fprintf(fw_agents, "%d,%d,", goal_row, goal_column);

	    sInt_32 start_vertex_id = m_start_configuration.get_AgentLocation(agent_id);

	    sInt_32 start_row = m_environment.calc_GridRow(start_vertex_id);
	    sInt_32 start_column = m_environment.calc_GridColumn(start_vertex_id);	    

	    fprintf(fw_agents, "%d,%d,", start_row, start_column);
	    fprintf(fw_agents, "\n");
	}	
    }    


    sResult sInstance::from_File_usc(const sString &map_filename, const sString &agents_filename)
    {
	sResult result;
	FILE *fr_map, *fr_agents;

	if ((fr_map = fopen(map_filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_USC_MAP_OPEN_ERROR;
	}
	if ((fr_agents = fopen(agents_filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_USC_AGNT_OPEN_ERROR;
	}
	
	result = from_Stream_usc(fr_map, fr_agents);
	
	if (sFAILED(result))
	{
	    fclose(fr_map);
	    fclose(fr_agents);	    
	    return result;
	}
	fclose(fr_map);
	fclose(fr_agents);	

	return sRESULT_SUCCESS;
    }


    sResult sInstance::from_Stream_usc(FILE *fr_map, FILE *fr_agents)
    {
	sResult result;
	sInt_32 N_Agents;

	if (sFAILED(result = m_environment.from_Stream_usc(fr_map)))
	{
	    return result;
	}
	fscanf(fr_agents, "%d\n", &N_Agents);

	m_start_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);
	m_goal_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{	    
	    sInt_32 goal_row, goal_column;
	    fscanf(fr_agents, "%d,%d,", &goal_row, &goal_column);

	    sInt_32 goal_vertex_id = m_environment.calc_GridVertexID(goal_row, goal_column);

	    sInt_32 init_row, init_column;
	    fscanf(fr_agents, "%d,%d,", &init_row, &init_column);
	    sInt_32 init_vertex_id =  m_environment.calc_GridVertexID(init_row, init_column);

	    float delay_ignore;
	    fscanf(fr_agents, "%f\n", &delay_ignore);	    

	    m_start_configuration.place_Agent(agent_id, init_vertex_id);
	    m_goal_configuration.place_Agent(agent_id, goal_vertex_id);
	}	
	
	return sRESULT_SUCCESS;
    }

    
    sResult sInstance::from_File_lusc(const sString &map_filename, const sString &agents_filename)
    {
	sResult result;
	FILE *fr_map, *fr_agents;

	if ((fr_map = fopen(map_filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_USC_MAP_OPEN_ERROR;
	}
	if ((fr_agents = fopen(agents_filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_USC_AGNT_OPEN_ERROR;
	}
	
	result = from_Stream_lusc(fr_map, fr_agents);
	
	if (sFAILED(result))
	{
	    fclose(fr_map);
	    fclose(fr_agents);	    
	    return result;
	}
	fclose(fr_map);
	fclose(fr_agents);	

	return sRESULT_SUCCESS;
    }


    sResult sInstance::from_Stream_lusc(FILE *fr_map, FILE *fr_agents)
    {
	sResult result;
	sInt_32 N_Agents;

	if (sFAILED(result = m_environment.from_Stream_lusc(fr_map)))
	{
	    return result;
	}
	fscanf(fr_agents, "%d\n", &N_Agents);

	m_start_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);
	m_goal_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);

	m_start_configuration.m_agent_Sizes.resize(N_Agents + 1);

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{	    
	    sInt_32 init_row, init_column;
	    fscanf(fr_agents, "%d,%d,", &init_row, &init_column);
	    sInt_32 init_vertex_id =  m_environment.calc_GridVertexID(init_row, init_column);

	    sInt_32 goal_row, goal_column;
	    fscanf(fr_agents, "%d,%d,", &goal_row, &goal_column);
	    sInt_32 goal_vertex_id = m_environment.calc_GridVertexID(goal_row, goal_column);	    

	    sInt_32 agent_size;
	    fscanf(fr_agents, "%d\n", &agent_size);
	    m_start_configuration.m_agent_Sizes[agent_id] = agent_size;

	    m_start_configuration.place_Agent(agent_id, init_vertex_id);
	    m_goal_configuration.place_Agent(agent_id, goal_vertex_id);
	}	
	
	return sRESULT_SUCCESS;
    }        


    sResult sInstance::to_File_dibox(const sString &filename) const
    {
	sResult result;
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_INSTANCE_DIBOX_OPEN_ERROR;
	}
	
	result = to_Stream_dibox(fw);
	if (sFAILED(result))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::to_Stream_dibox(FILE *fw) const
    {
	sInt_32 N_Agents = m_start_configuration.get_AgentCount();

	m_environment.to_Stream_dibox(fw);
	fprintf(fw, "LIST OF AGENTS:%d\n", N_Agents);
	
	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 init_vertex_id, goal_vertex_id;

	    init_vertex_id = m_start_configuration.get_AgentLocation(agent_id);
	    goal_vertex_id = m_goal_configuration.get_AgentLocation(agent_id);
	    
	    fprintf(fw, "(q%d,%d,%d)\n", agent_id, init_vertex_id + 1, goal_vertex_id + 1);
	}

	return sRESULT_SUCCESS;
    }
    
    
    sResult sInstance::from_File_dibox(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_DIBOX_OPEN_ERROR;
	}
	
	result = from_Stream_dibox(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sInstance::from_Stream_dibox(FILE *fr)
    {
	sResult result;
	sInt_32 N_Agents;

	if (sFAILED(result = m_environment.from_Stream_dibox(fr)))
	{
	    return result;
	}
	fscanf(fr, "LIST OF AGENTS:%d\n", &N_Agents);
	
	m_start_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);
	m_goal_configuration = sConfiguration(m_environment.get_VertexCount(), N_Agents);

	for (sInt_32 agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    sInt_32 init_vertex_id, goal_vertex_id;

	    while(fgetc(fr) != ',');
	    fscanf(fr, "%d,%d)\n", &init_vertex_id, &goal_vertex_id);
	    
	    m_start_configuration.place_Agent(agent_id, init_vertex_id - 1);
	    m_goal_configuration.place_Agent(agent_id, goal_vertex_id - 1);
	}

	return sRESULT_SUCCESS;
    }
    

    sResult sInstance::from_File_movi(const sString &filename, const sUndirectedGraph &environment, sInt_32 N_agents)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_INSTANCE_MOVISCEN_OPEN_ERROR;
	}
	
	if (sFAILED(result = from_Stream_movi(fr, environment, N_agents)))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }

    
    sResult sInstance::from_Stream_movi(FILE *fr, const sUndirectedGraph &environment, sInt_32 N_agents)
    {
	sInt_32 version_unused;		
	fscanf(fr, "version %d\n", &version_unused);

	sInt_32 N_Agents_1 = 1;

	if (N_agents < 0)
	{
	    N_agents = 0;
	    
	    while (!feof(fr))
	    {
		sInt_32 number_ignore;
		sChar map_name_ignore[128];
		
		sInt_32 x_size, y_size;
		
		sInt_32 x_start, y_start;
		sInt_32 x_goal, y_goal;
		
		sDouble real_number_ignore;
		
		fscanf(fr, "%d %s %d %d %d %d %d %d %lf\n", &number_ignore, map_name_ignore, &x_size, &y_size, &x_start, &y_start, &x_goal, &y_goal, &real_number_ignore);
		++N_agents;
	    }
	    
	    if (fseek(fr, 0, SEEK_SET) != 0)
	    {
		return sAGENT_INSTANCE_SEEK_ERROR;
	    }
	}

	m_start_configuration = sConfiguration(environment.get_VertexCount(), N_agents);
	m_goal_configuration = sConfiguration(environment.get_VertexCount(), N_agents);	    	
	
	while (!feof(fr) && N_agents > 0)
	{
	    sInt_32 number_ignore;
	    sChar map_name_ignore[128];

	    sInt_32 x_size, y_size;

	    sInt_32 x_start, y_start;
	    sInt_32 x_goal, y_goal;

	    sDouble real_number_ignore;
	    
	    fscanf(fr, "%d %s %d %d %d %d %d %d %lf\n", &number_ignore, map_name_ignore, &x_size, &y_size, &x_start, &y_start, &x_goal, &y_goal, &real_number_ignore);

	    sInt_32 start_location_id = environment.m_Matrix[y_start * x_size + x_start];
	    m_start_configuration.place_Agent(N_Agents_1, start_location_id);

	    sInt_32 goal_location_id = environment.m_Matrix[y_goal * x_size + x_goal];
	    m_goal_configuration.place_Agent(N_Agents_1, goal_location_id);	    
	    	    
	    ++N_Agents_1;
	    --N_agents;
	}	
	return sRESULT_SUCCESS;
    }    

    
/*----------------------------------------------------------------------------*/
// sSolution

    const sSolution::Move sSolution::UNDEFINED_MOVE = sSolution::Move(0, 0, 0);


/*----------------------------------------------------------------------------*/

    sSolution::Move::Move(sInt_32 agent_id, sInt_32 src_vrtx_id, sInt_32 dest_vrtx_id)
	: m_agent_id(agent_id)
	, m_src_vrtx_id(src_vrtx_id)
	, m_dest_vrtx_id(dest_vrtx_id)
	, m_crt_time(0)
    {
	// nothing
    }


    sSolution::Move::Move(sInt_32 agent_id, sInt_32 src_vrtx_id, sInt_32 dest_vrtx_id, sInt_32 crt_time)
	: m_agent_id(agent_id)
	, m_src_vrtx_id(src_vrtx_id)
	, m_dest_vrtx_id(dest_vrtx_id)
	, m_crt_time(crt_time)
    {
	// nothing
    }


    bool sSolution::Move::is_Undefined(void) const
    {
	return (m_agent_id <= 0);
    }


    bool sSolution::Move::is_Dependent(const Move &move) const
    {
	return (   m_src_vrtx_id == move.m_src_vrtx_id
		|| m_src_vrtx_id == move.m_dest_vrtx_id
		|| m_dest_vrtx_id == move.m_src_vrtx_id
		|| m_dest_vrtx_id == move.m_dest_vrtx_id);
    }



/*----------------------------------------------------------------------------*/

    sSolution::Step::Step(sInt_32 time)
	: m_time(time)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sSolution::sSolution()
	: m_Moves_cnt(0)
	, m_optimality_ratio(-1.0)
    {
	// nothing
    }


    sSolution::sSolution(sInt_32 start_step, const sSolution &sub_solution)
	: m_Moves_cnt(0)
	, m_optimality_ratio(-1.0)
    {
	sInt_32 N_Steps = sub_solution.m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = sub_solution.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i + start_step, *move);
	    }
	}	
    }


    sSolution::sSolution(const sSolution &sub_solution_1, const sSolution sub_solution_2)
	: m_Moves_cnt(0)
	, m_optimality_ratio(-1.0)
    {
	sInt_32 N_Steps_1 = sub_solution_1.m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps_1; ++i)
	{
	    const Step &step = sub_solution_1.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i, *move);
	    }
	}

	sInt_32 N_Steps_2 = sub_solution_2.m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps_2; ++i)
	{
	    const Step &step = sub_solution_2.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i, *move);
	    }
	}
    }

    bool sSolution::is_Null(void) const
    {
	return (m_Steps.empty());
    }


    sInt_32 sSolution::get_MoveCount(void) const
    {
	return m_Moves_cnt;
    }


    sInt_32 sSolution::get_StepCount(void) const
    {
	return m_Steps.size();
    }


    void sSolution::add_Move(sInt_32 time, const Move &move)
    {
	sInt_32 push_cnt = time - m_Steps.size();

	while (push_cnt-- >= 0)
	{
	    m_Steps.push_back(Step(m_Steps.size()));
	}
	m_Steps[time].m_Moves.push_back(move);
	++m_Moves_cnt;
    }


    sInt_32 sSolution::calc_EmptySteps(void) const
    {
	sInt_32 N_empty_Steps = 0;
	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    if (step.m_Moves.empty())
	    {
		++N_empty_Steps;
	    }
	}
	return N_empty_Steps;
    }


    void sSolution::remove_EmptySteps(void)
    {
	sInt_32 time = 0;
	Steps_vector clean_Steps;

	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    Step &step = m_Steps[i];

	    if (!step.m_Moves.empty())
	    {
		step.m_time = time++;
		clean_Steps.push_back(step);
	    }
	}
	m_Steps = clean_Steps;
    }


    sSolution sSolution::extract_Subsolution(sInt_32 start_step, sInt_32 final_step) const
    {
	sSolution subsolution;

	for (sInt_32 i = start_step; i <= final_step; ++i)
	{
	    const Step &step = m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		subsolution.add_Move(i, *move);
	    }
	}	

	return subsolution;
    }


    void sSolution::execute_Solution(const sConfiguration &start_configuration,
				     sConfiguration       &final_configuration,
				     sInt_32                   N_Steps) const
    {
	final_configuration = start_configuration;

	if (N_Steps == N_STEPS_UNDEFINED)
	{
	    for (sSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    final_configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
		}	    
	    }
	}
	else
	{
	    for (sSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    final_configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
		}	    
		if (--N_Steps <= 0)
		{
		    return;
		}
	    }
	}
    }


    void sSolution::execute_Step(const sConfiguration &current_configuration,
				 sConfiguration       &final_configuration,
				 sInt_32                   step_idx) const
    {
	final_configuration = current_configuration;

	const Step &step = m_Steps[step_idx];

	for (sSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    final_configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
	}	    
    }


    void sSolution::execute_Solution(sConfiguration &configuration, sInt_32 N_Steps) const
    {
	if (N_Steps == N_STEPS_UNDEFINED)
	{
	    for (sSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
		}	    
	    }
	}
	else
	{
	    for (sSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
		}	    
		if (--N_Steps <= 0)
		{
		    return;
		}
	    }
	}
    }


    void sSolution::execute_Step(sConfiguration &configuration, sInt_32 step_idx) const
    {
	 const Step &step = m_Steps[step_idx];

	for (sSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
	}	    
    }


    bool sSolution::verify_Step(const sConfiguration &configuration, sInt_32 step_idx) const
    {
	const Step &step = m_Steps[step_idx];

	for (sSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    if (!configuration.verify_Move(move->m_agent_id, move->m_dest_vrtx_id))
	    {
		return false;
	    }
	}
	return true;
    }


    bool sSolution::check_Step(const sConfiguration &configuration, sInt_32 step_idx) const
    {
	const Step &step = m_Steps[step_idx];

	for (sSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    if (!configuration.check_Move(move->m_agent_id, move->m_dest_vrtx_id))
	    {
		return false;
	    }
	}
	return true;
    }


    void sSolution::filter_Solution(const sConfiguration &start_configuration,
				    const sConfiguration &goal_configuration,
				    sSolution            &filter_solution) const
    {
	sConfiguration agent_configuration = start_configuration;

	for (sSolution::Steps_vector::const_iterator step = m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		agent_configuration.move_Agent(move->m_agent_id, move->m_dest_vrtx_id);
		filter_solution.add_Move(step->m_time, *move);

		if (goal_configuration == agent_configuration)
		{
		    break;
		}
	    }	    
	}
    }


    sInt_32 sSolution::calc_CriticalTimes(void)
    {
	sInt_32 max_crt_time = 0;

	for (sSolution::Steps_vector::iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sSolution::Moves_list::iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		move->m_crt_time = 0;

		for (sSolution::Steps_vector::const_iterator prev_step =  m_Steps.begin(); prev_step != step; ++prev_step)
		{
		    for (sSolution::Moves_list::const_iterator prev_move = prev_step->m_Moves.begin(); prev_move != prev_step->m_Moves.end(); ++prev_move)
		    {
			if (move->is_Dependent(*prev_move))
			{
			    if (prev_move->m_crt_time >= move->m_crt_time)
			    {
				move->m_crt_time = prev_move->m_crt_time + 1;
				max_crt_time = sMAX(max_crt_time, move->m_crt_time);
			    }
			}
		    }
		}
		for (sSolution::Moves_list::const_iterator prev_move = step->m_Moves.begin(); prev_move != move; ++prev_move)
		{
		    if (move->is_Dependent(*prev_move))
		    {
			if (prev_move->m_crt_time >= move->m_crt_time)
			{
			    move->m_crt_time = prev_move->m_crt_time + 1;
			    max_crt_time = sMAX(max_crt_time, move->m_crt_time);
			}
		    }
		}
	    }
	}

	return max_crt_time;
    }


    void sSolution::criticalize_Solution(sSolution &critical_solution)
    {
	calc_CriticalTimes();

	for (sSolution::Steps_vector::iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sSolution::Moves_list::iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		critical_solution.add_Move(move->m_crt_time, *move);
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    void sSolution::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sSolution::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sMuliagent solution: (|moves| = %d, paralellism = %.3f) [\n", indent.c_str(), m_Moves_cnt, (double)m_Moves_cnt / m_Steps.size());

	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    fprintf(fw, "%s%sStep %d: ", indent.c_str(), s_INDENT.c_str(), step.m_time);

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%d#%d->%d ", move->m_agent_id, move->m_src_vrtx_id, move->m_dest_vrtx_id);
	    }
	    fprintf(fw, "\n");
	}

	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sSolution::to_Screen(const sUndirectedGraph &grid, const sString &indent) const
    {
	to_Stream(grid, stdout, indent);
    }


    void sSolution::to_Stream(const sUndirectedGraph &grid, FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sMuliagent solution: (|moves| = %d, paralellism = %.3f) [\n", indent.c_str(), m_Moves_cnt, (double)m_Moves_cnt / m_Steps.size());

	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    fprintf(fw, "%s%sStep %d: ", indent.c_str(), s_INDENT.c_str(), step.m_time);

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		sInt_32 src_row = grid.calc_GridRow(move->m_src_vrtx_id);
		sInt_32 src_column = grid.calc_GridColumn(move->m_src_vrtx_id);
		sInt_32 dest_row = grid.calc_GridRow(move->m_dest_vrtx_id);
		sInt_32 dest_column = grid.calc_GridColumn(move->m_dest_vrtx_id);
		    
		fprintf(fw, "%d#%d[%d,%d] --> %d[%d,%d] ", move->m_agent_id, move->m_src_vrtx_id, src_row, src_column, move->m_dest_vrtx_id, dest_row, dest_column);
	    }
	    fprintf(fw, "\n");
	}

	fprintf(fw, "%s]\n", indent.c_str());
    }


    sResult sSolution::to_File(const sUndirectedGraph &grid, const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	to_Stream(grid, fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sSolution::to_File_cpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_cpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sSolution::to_Stream_cpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sFine solution\n", indent.c_str());
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%s%d # %d ---> %d (%d)\n", indent.c_str(), move->m_agent_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
    }



    sResult sSolution::to_File_ccpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_ccpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sSolution::to_Stream_ccpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sFine solution\n", indent.c_str());
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%s%d # %d ---> %d (%d)\n", indent.c_str(), move->m_agent_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
    }    


    sResult sSolution::to_File_mpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_mpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sSolution::to_Stream_mpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sFine solution\n", indent.c_str());
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%s%d # %d ---> %d (%d)\n", indent.c_str(), move->m_agent_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
    }


    sResult sSolution::to_File_cmpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_cmpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sSolution::to_Stream_cmpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sFine solution\n", indent.c_str());
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%s%d # %d ---> %d (%d)\n", indent.c_str(), move->m_agent_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
    }        


    sResult sSolution::to_File_graphrec(const sString &filename, const sInstance &instance, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_graphrec(fw, instance, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sSolution::to_Stream_graphrec(FILE *fw, const sInstance &instance, const sString &indent) const
    {
	fprintf(fw, "id:-\n");
	instance.to_Stream_cpf(fw, indent);
	
	fprintf(fw, "Solution\n");
	sInt_32 N_Steps = m_Steps.size();
	for (sInt_32 i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%d ---> %d (%d)\n", move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
    }    


    sResult sSolution::from_File_cpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	result = from_Stream_cpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }    


    sResult sSolution::from_Stream_cpf(FILE *fr)
    {
	sInt_32 N_Moves;
	sInt_32 c = fgetc(fr);

	while (c != 'F')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, "Fine solution\nLength:%d\n", &N_Moves);
        #ifdef sDEBUG
	printf("Length:%d\n", N_Moves);
	#endif

	for (sInt_32 i = 0; i < N_Moves; ++i)
	{
	    sInt_32 agent_id, src_vertex_id, dest_vertex_id, step;
	    fscanf(fr, "%d # %d ---> %d (%d)\n", &agent_id, &src_vertex_id, &dest_vertex_id, &step);
	    #ifdef sDEBUG
	    printf("%d # %d ---> %d (%d)\n", agent_id, src_vertex_id, dest_vertex_id, step);
	    #endif
	    add_Move(step, Move(agent_id, src_vertex_id, dest_vertex_id));
	}

	return sRESULT_SUCCESS;
    }


    sResult sSolution::from_File_ccpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	result = from_Stream_ccpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }    


    sResult sSolution::from_Stream_ccpf(FILE *fr)
    {
	sInt_32 N_Moves;
	sInt_32 c = fgetc(fr);

	while (c != 'F')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, "Fine solution\nLength:%d\n", &N_Moves);
        #ifdef sDEBUG
	printf("Length:%d\n", N_Moves);
	#endif

	for (sInt_32 i = 0; i < N_Moves; ++i)
	{
	    sInt_32 agent_id, src_vertex_id, dest_vertex_id, step;
	    fscanf(fr, "%d # %d ---> %d (%d)\n", &agent_id, &src_vertex_id, &dest_vertex_id, &step);
	    #ifdef sDEBUG
	    printf("%d # %d ---> %d (%d)\n", agent_id, src_vertex_id, dest_vertex_id, step);
	    #endif
	    add_Move(step, Move(agent_id, src_vertex_id, dest_vertex_id));
	}

	return sRESULT_SUCCESS;
    }    


    sResult sSolution::from_File_mpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	result = from_Stream_mpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sSolution::from_Stream_mpf(FILE *fr)
    {
	sInt_32 N_Moves;
	sInt_32 c = fgetc(fr);

	while (c != 'F')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, "Fine solution\nLength:%d\n", &N_Moves);
        #ifdef sDEBUG
	printf("Length:%d\n", N_Moves);
	#endif

	for (sInt_32 i = 0; i < N_Moves; ++i)
	{
	    sInt_32 agent_id, src_vertex_id, dest_vertex_id, step;
	    fscanf(fr, "%d # %d ---> %d (%d)\n", &agent_id, &src_vertex_id, &dest_vertex_id, &step);
	    #ifdef sDEBUG
	    printf("%d # %d ---> %d (%d)\n", agent_id, src_vertex_id, dest_vertex_id, step);
	    #endif
	    add_Move(step, Move(agent_id, src_vertex_id, dest_vertex_id));
	}

	return sRESULT_SUCCESS;
    }


    sResult sSolution::from_File_cmpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sAGENT_SOLUTION_OPEN_ERROR;
	}
	
	result = from_Stream_cmpf(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sSolution::from_Stream_cmpf(FILE *fr)
    {
	sInt_32 N_Moves;
	sInt_32 c = fgetc(fr);

	while (c != 'F')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, "Fine solution\nLength:%d\n", &N_Moves);
        #ifdef sDEBUG
	printf("Length:%d\n", N_Moves);
	#endif

	for (sInt_32 i = 0; i < N_Moves; ++i)
	{
	    sInt_32 agent_id, src_vertex_id, dest_vertex_id, step;
	    fscanf(fr, "%d # %d ---> %d (%d)\n", &agent_id, &src_vertex_id, &dest_vertex_id, &step);
	    #ifdef sDEBUG
	    printf("%d # %d ---> %d (%d)\n", agent_id, src_vertex_id, dest_vertex_id, step);
	    #endif
	    add_Move(step, Move(agent_id, src_vertex_id, dest_vertex_id));
	}

	return sRESULT_SUCCESS;
    }        


/*----------------------------------------------------------------------------*/

} // namespace boOX
