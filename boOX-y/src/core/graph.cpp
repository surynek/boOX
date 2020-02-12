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
/* graph.cpp / 1-224_leibniz                                                  */
/*----------------------------------------------------------------------------*/
//
// Graph related data structures and algorithms.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "result.h"

#include "common/types.h"
#include "core/graph.h"

using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sVertex

    sVertex::sVertex()
	: m_id(0)
	, m_distance(0)
	, m_prev_id(-1)
	, m_capacity(1)
    {
	// nothing
    }


    sVertex::sVertex(sInt_32 id)
	: m_id(id)
	, m_distance(0)
	, m_prev_id(-1)
	, m_capacity(1)
    {
	// nothing
    }

    
    sVertex::sVertex(sInt_32 id, sInt_32 capacity)
	: m_id(id)
	, m_distance(0)
	, m_prev_id(-1)
	, m_capacity(capacity)
    {
	// nothing
    }    


    sVertex::sVertex(const sVertex &vertex)
	: m_id(vertex.m_id)
	, m_distance(vertex.m_distance)
	, m_prev_id(vertex.m_prev_id)
	, m_capacity(vertex.m_capacity)
	, m_Conflicts(vertex.m_Conflicts)
    {	
	sASSERT(m_Neighbors.empty() && vertex.m_Neighbors.empty());
    }


    const sVertex& sVertex::operator=(const sVertex &vertex)
    {
	m_id = vertex.m_id;
	m_capacity = vertex.m_capacity;	
	m_Conflicts = vertex.m_Conflicts;
	sASSERT(m_Neighbors.empty() && vertex.m_Neighbors.empty());

	return *this;
    }


/*----------------------------------------------------------------------------*/

    sInt_32 sVertex::calc_NeighborCount(void) const
    {
	return m_Neighbors.size();
    }


    sInt_32 sVertex::calc_NeighborOrder(sInt_32 vertex_id) const
    {
	sInt_32 order = 0;
	for (Neighbors_list::const_iterator neighbor = m_Neighbors.begin(); neighbor != m_Neighbors.end(); ++neighbor)
	{
	    if ((*neighbor)->m_target->m_id == vertex_id)
	    {
		return order; 
	    }
	    ++order;
	}

	return ORDER_UNDEFINED;
    }


    sInt_32 sVertex::calc_NeighborID(sInt_32 order) const
    {
	Neighbors_list::const_iterator neighbor = m_Neighbors.begin();

	while (--order >= 0)
	{
	    ++neighbor;
	}

	return (*neighbor)->m_target->m_id;
    }


/*----------------------------------------------------------------------------*/
    
    void sVertex::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sVertex::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sVertex: (id = %d, distance = %d, prev_id = %d, capacity = %d)", indent.c_str(), m_id, m_distance, m_prev_id, m_capacity);
	
	if (!m_Neighbors.empty())
	{
	    fprintf(fw, " {");
	    for (Neighbors_list::const_iterator neighbor = m_Neighbors.begin(); neighbor != m_Neighbors.end(); ++neighbor)
	    {
		fprintf(fw, "%d->%d ", (*neighbor)->m_source->m_id, (*neighbor)->m_target->m_id);
	    }
	    fprintf(fw, "}");

	    fprintf(fw, " i[");
	    for (Neighbors_list::const_iterator neighbor = m_in_Neighbors.begin(); neighbor != m_in_Neighbors.end(); ++neighbor)
	    {
		fprintf(fw, "%d ", (*neighbor)->m_source->m_id);
	    }
	    fprintf(fw, "]");

	    fprintf(fw, " o[");
	    for (Neighbors_list::const_iterator neighbor = m_out_Neighbors.begin(); neighbor != m_out_Neighbors.end(); ++neighbor)
	    {
		fprintf(fw, "%d ", (*neighbor)->m_target->m_id);
	    }
	    fprintf(fw, "]");
	}
	if (!m_Conflicts.empty())
	{
	    fprintf(fw, " < ");
	    for (VertexIDs_vector::const_iterator conflict = m_Conflicts.begin(); conflict != m_Conflicts.end(); ++conflict)
	    {
		fprintf(fw, "%d ", *conflict);
	    }
	    fprintf(fw, ">");	    
	}
	fprintf(fw, "\n");
    }


/*----------------------------------------------------------------------------*/
// sArc

    sArc::sArc()
	: m_edge(NULL)
	, m_source(NULL)
	, m_target(NULL)
    {
	// nothing
    }


    sArc::sArc(sEdge *edge, sVertex *source, sVertex *target)
	: m_edge(edge)
	, m_source(source)
	, m_target(target)
    {
	// nothing
    }


    sArc::sArc(const sArc &arc)
	: m_edge(arc.m_edge)
	, m_source(arc.m_source)
	, m_target(arc.m_target)
    {
	// nothing
    }


    const sArc& sArc::operator=(const sArc &arc)
    {
	m_edge = arc.m_edge;
	m_source = arc.m_source;
	m_target = arc.m_target;

	return *this;
    }


    void sArc::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

	 
    void sArc::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sArc: (edge = %p, source = %p, target = %p)\n", indent.c_str(), (void*)m_edge, (void*)m_source, (void*)m_target);
    }


/*----------------------------------------------------------------------------*/
// sEdge

    sEdge::sEdge(sInt_32 id, sVertex *vertex_u, sVertex *vertex_v, bool directed)
	: m_id(id)
	, m_directed(directed)
	, m_arc_uv(this, vertex_u, vertex_v)
	, m_arc_vu(this, vertex_v, vertex_u)
    {
	// nothing
    }


    sEdge::sEdge(const sEdge &edge)
	: m_id(edge.m_id)
	, m_directed(edge.m_directed)
	, m_arc_uv(this, edge.m_arc_uv.m_source, edge.m_arc_uv.m_target)
	, m_arc_vu(this, edge.m_arc_vu.m_source, edge.m_arc_vu.m_target)
    {
	// nothing
    }


    const sEdge& sEdge::operator=(const sEdge &edge)
    {
	m_id = edge.m_id;
	m_directed = edge.m_directed;

	m_arc_uv = sArc(this, edge.m_arc_uv.m_source, edge.m_arc_uv.m_target);
	m_arc_vu = sArc(this, edge.m_arc_vu.m_source, edge.m_arc_vu.m_target);

	return *this;
    }


    void sEdge::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sEdge::to_Stream(FILE *fw, const sString &indent) const
    {
	if (m_directed)
	{
	    fprintf(fw, "%sEdge  %d: %d --> %d", indent.c_str(), m_id, m_arc_uv.m_source->m_id, m_arc_uv.m_target->m_id);
	}
	else
	{
	    fprintf(fw, "%sEdge  %d: %d <-> %d", indent.c_str(), m_id, m_arc_uv.m_source->m_id, m_arc_uv.m_target->m_id);
	}
	fprintf(fw, "\n");
	
/*
	fprintf(fw, "%s]\n", indent.c_str());
	m_arc_uv.to_Stream(fw, indent + sRELOC_INDENT);
	m_arc_vu.to_Stream(fw, indent + sRELOC_INDENT);
	fprintf(fw, "%s]\n", indent.c_str());
*/
    }


/*----------------------------------------------------------------------------*/
// sUndirectedGraph

    sUndirectedGraph::sUndirectedGraph()
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	// nothing
    }


    sUndirectedGraph::sUndirectedGraph(bool directed)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	// nothing
    }


    sUndirectedGraph::sUndirectedGraph(sInt_32 N_vertices, double edge_probability)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
       	add_Vertices(N_vertices);

	for (sInt_32 u_id = 0; u_id < N_vertices - 1; ++u_id)
	{
	    for (sInt_32 v_id = u_id + 1; v_id < N_vertices; ++v_id)
	    {
		double rnd = (double)rand() / RAND_MAX;
		if (rnd <= edge_probability)
		{
		    add_Edge(u_id, v_id);
		}
	    }
	}
    }


    sUndirectedGraph::sUndirectedGraph(sInt_32 x_size, sInt_32 y_size)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	sInt_32 grid_size = x_size * y_size;
	m_Matrix = new int[grid_size];

	for (sInt_32 i = 0; i < grid_size; ++i)
	{
	    m_Matrix[i] = i + 1;
	}
	add_Vertices(x_size * y_size);
	
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		add_Edge(u_id, v_id);
		add_Edge(u_id, w_id);		
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    add_Edge(u_id, v_id);
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    add_Edge(u_id, v_id);
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(sInt_32 x_size, sInt_32 y_size, double obstacle_prob)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	sInt_32 rnd_limit = RAND_MAX * obstacle_prob;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		sInt_32 rnd = rand();

		if (rnd > rnd_limit)
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
	    }
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(sInt_32 x_size, sInt_32 y_size, sInt_32 N_obstacles)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	sInt_32 matrix_size = x_size * y_size;
	m_Matrix = new int[matrix_size];
	sInt_32 *selection_Matrix = new int[matrix_size];

	for (sInt_32 i = 0; i < matrix_size; ++i)	
	{
	    m_Matrix[i] = 0;
	    selection_Matrix[i] = i;
	}

	sInt_32 remaining = matrix_size;
	for (sInt_32 i = 0; i < N_obstacles; ++i)
	{
	    sInt_32 rnd = rand() % remaining;
	    m_Matrix[selection_Matrix[rnd]] = -1;

	    selection_Matrix[rnd] = selection_Matrix[remaining - 1];
	    --remaining;
	}
	delete selection_Matrix;

	sInt_32 cnt = 0;
	for (sInt_32 i = 0; i < matrix_size; ++i)	
	{
	    if (m_Matrix[i] != -1)
	    {
		m_Matrix[i] = cnt++;
	    }
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, sInt_32 x_size, sInt_32 y_size)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	sInt_32 grid_size = x_size * y_size;
	m_Matrix = new int[grid_size];

	for (sInt_32 i = 0; i < grid_size; ++i)
	{
	    m_Matrix[i] = i + 1;
	}
	add_Vertices(x_size * y_size);
	
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (directed)
		{		    
		    add_RandomArrow(u_id, v_id);
		    add_RandomArrow(u_id, w_id);
		}
		else
		{
		    add_Edge(u_id, v_id);
		    add_Edge(u_id, w_id);		    
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (directed)
	    {
		add_RandomArrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (directed)
	    {
		add_RandomArrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, sInt_32 x_size, sInt_32 y_size, double obstacle_prob)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	sInt_32 rnd_limit = RAND_MAX * obstacle_prob;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		sInt_32 rnd = rand();

		if (rnd > rnd_limit)
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
	    }
	}

	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			if (directed)
			{			    
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
			}
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			if (directed)
			{
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[w_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
			}
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, sInt_32 x_size, sInt_32 y_size, sInt_32 N_obstacles)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	sInt_32 matrix_size = x_size * y_size;
	m_Matrix = new int[matrix_size];
	sInt_32 *selection_Matrix = new int[matrix_size];

	for (sInt_32 i = 0; i < matrix_size; ++i)	
	{
	    m_Matrix[i] = 0;
	    selection_Matrix[i] = i;
	}

	sInt_32 remaining = matrix_size;
	for (sInt_32 i = 0; i < N_obstacles; ++i)
	{
	    sInt_32 rnd = rand() % remaining;
	    m_Matrix[selection_Matrix[rnd]] = -1;

	    selection_Matrix[rnd] = selection_Matrix[remaining - 1];
	    --remaining;
	}
	delete selection_Matrix;

	sInt_32 cnt = 0;
	for (sInt_32 i = 0; i < matrix_size; ++i)	
	{
	    if (m_Matrix[i] != -1)
	    {
		m_Matrix[i] = cnt++;
	    }
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			if (directed)
			{
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
			}
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			if (directed)
			{
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[w_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
			}
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}		    
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	initialize_InverseMatrix();
    }    


    sUndirectedGraph::sUndirectedGraph(sInt_32 base_cycle_size, sInt_32 ear_min_size, sInt_32 ear_max_size, sInt_32 graph_size)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	Edges_vector Edges;

	sInt_32 vertex_id = 0;
	add_Vertex();
	++vertex_id;

	for (sInt_32 i = 1; i < base_cycle_size; ++i)
	{
	    vertex_id++;
	    add_Vertex();
	    Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	}
	Edges.push_back(Edge(vertex_id - 1, 0));
	
	while (vertex_id < graph_size)
	{
	    sInt_32 ear_size = ear_min_size + rand() % (ear_max_size - ear_min_size);
	    sInt_32 first_connection_id = rand() % vertex_id;
	    sInt_32 last_connection_id = rand() % vertex_id;

	    if (last_connection_id == first_connection_id)
	    {
		last_connection_id = (last_connection_id + 1) % vertex_id;
	    }

	    sInt_32 first_vertex_id = ++vertex_id;
	    add_Vertex();

	    for (sInt_32 i = 1; i < ear_size; ++i)
	    {
		vertex_id++;
		add_Vertex();
		Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	    }
	    sInt_32 last_vertex_id = vertex_id - 1;

	    Edges.push_back(Edge(first_vertex_id, first_connection_id));
	    Edges.push_back(Edge(last_vertex_id, last_connection_id));
	}
	for (Edges_vector::const_iterator edge = Edges.begin(); edge != Edges.end(); ++edge)
	{
	    add_Edge(edge->m_u_id, edge->m_v_id);
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, sInt_32 base_cycle_size, sInt_32 ear_min_size, sInt_32 ear_max_size, sInt_32 graph_size)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
    {
	Edges_vector Edges;

	sInt_32 vertex_id = 0;
	add_Vertex();
	++vertex_id;

	for (sInt_32 i = 1; i < base_cycle_size; ++i)
	{
	    add_Vertex();
	    ++vertex_id;
	    Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	}
	Edges.push_back(Edge(vertex_id - 1, 0));
	
	while (vertex_id < graph_size)
	{
	    sInt_32 ear_size = ear_min_size + rand() % (ear_max_size - ear_min_size);
	    sInt_32 first_connection_id = rand() % vertex_id;
	    sInt_32 last_connection_id = rand() % vertex_id;

	    if (last_connection_id == first_connection_id)
	    {
		last_connection_id = (last_connection_id + 1) % vertex_id;
	    }

	    sInt_32 first_vertex_id = vertex_id;
	    add_Vertex();
	    ++vertex_id;

	    for (sInt_32 i = 1; i < ear_size; ++i)
	    {
		add_Vertex();
		++vertex_id;
		Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	    }
	    sInt_32 last_vertex_id = vertex_id - 1;

	    Edges.push_back(Edge(first_connection_id, first_vertex_id));
	    Edges.push_back(Edge(last_vertex_id, last_connection_id));
	}
	for (Edges_vector::const_iterator edge = Edges.begin(); edge != Edges.end(); ++edge)
	{
	    add_Arrow(edge->m_u_id, edge->m_v_id);
	}
	initialize_InverseMatrix();
    }    


    sUndirectedGraph::sUndirectedGraph(const sUndirectedGraph &undirected_graph)
	: m_directed(undirected_graph.m_directed)
	, m_Edge_cnt(0)
	, m_x_size(undirected_graph.m_x_size)
	, m_y_size(undirected_graph.m_y_size)
    {
	if (undirected_graph.m_Matrix != NULL)
	{
	    m_Matrix = new int[undirected_graph.m_x_size * undirected_graph.m_y_size];
	    
	    sInt_32 grid_size = undirected_graph.m_x_size * undirected_graph.m_y_size;
	    for (sInt_32 i = 0; i < grid_size; ++i)
	    {
		m_Matrix[i] = undirected_graph.m_Matrix[i];
	    }
	}
	else
	{
	    m_Matrix = NULL;
	}
	m_inverse_Matrix = undirected_graph.m_inverse_Matrix;
	
	add_Vertices(undirected_graph.m_Vertices.size());
	for (sInt_32 i = 0; i < undirected_graph.m_Vertices.size(); ++i)
	{
	    m_Vertices[i].m_Conflicts = undirected_graph.m_Vertices[i].m_Conflicts;
	    m_Vertices[i].m_capacity = undirected_graph.m_Vertices[i].m_capacity;	    
	}

	for (Edges_list::const_iterator edge = undirected_graph.m_Edges.begin(); edge != undirected_graph.m_Edges.end(); ++edge)	
	{
	    if (edge->m_directed)
	    {
		add_Arrow(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    else
	    {
		add_Edge(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	}

	m_all_pairs_distances_calculated = undirected_graph.m_all_pairs_distances_calculated;
	m_all_pairs_Distances = undirected_graph.m_all_pairs_Distances;

	m_source_goal_distances_calculated = undirected_graph.m_source_goal_distances_calculated;
	m_source_Distances = undirected_graph.m_source_Distances;
	m_goal_Distances = undirected_graph.m_goal_Distances;
    }


    sUndirectedGraph::sUndirectedGraph(const sUndirectedGraph &undirected_graph, bool opposite)
	: m_directed(undirected_graph.m_directed)
	, m_Edge_cnt(0)
	, m_x_size(undirected_graph.m_x_size)
	, m_y_size(undirected_graph.m_y_size)
    {
	if (undirected_graph.m_Matrix != NULL)
	{
	    m_Matrix = new int[undirected_graph.m_x_size * undirected_graph.m_y_size];
	    
	    sInt_32 grid_size = undirected_graph.m_x_size * undirected_graph.m_y_size;
	    for (sInt_32 i = 0; i < grid_size; ++i)
	    {
		m_Matrix[i] = undirected_graph.m_Matrix[i];
	    }
	}
	else
	{
	    m_Matrix = NULL;
	}
	m_inverse_Matrix = undirected_graph.m_inverse_Matrix;
		
	add_Vertices(undirected_graph.m_Vertices.size());
	for (sInt_32 i = 0; i < undirected_graph.m_Vertices.size(); ++i)
	{
	    m_Vertices[i].m_Conflicts = undirected_graph.m_Vertices[i].m_Conflicts;
	    m_Vertices[i].m_capacity = undirected_graph.m_Vertices[i].m_capacity;	    
	}	

	for (Edges_list::const_iterator edge = undirected_graph.m_Edges.begin(); edge != undirected_graph.m_Edges.end(); ++edge)	
	{
	    if (edge->m_directed)
	    {
		if (opposite)
		{
		    add_Arrow(edge->m_arc_uv.m_target->m_id, edge->m_arc_uv.m_source->m_id);
		}
		else
		{
		    add_Arrow(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
		}
	    }
	    else
	    {
		add_Edge(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	}

	m_all_pairs_distances_calculated = undirected_graph.m_all_pairs_distances_calculated;
	m_all_pairs_Distances = undirected_graph.m_all_pairs_Distances;

	m_source_goal_distances_calculated = undirected_graph.m_source_goal_distances_calculated;
	m_source_Distances = undirected_graph.m_source_Distances;
	m_goal_Distances = undirected_graph.m_goal_Distances;
    }    


    const sUndirectedGraph& sUndirectedGraph::operator=(const sUndirectedGraph &undirected_graph)
    {
	m_directed = undirected_graph.m_directed;
	m_Edge_cnt = 0;

	m_x_size = undirected_graph.m_x_size;
	m_y_size = undirected_graph.m_y_size;

	if (undirected_graph.m_Matrix != NULL)
	{
	    m_Matrix = new int[undirected_graph.m_x_size * undirected_graph.m_y_size];
	    
	    sInt_32 grid_size = undirected_graph.m_x_size * undirected_graph.m_y_size;
	    for (sInt_32 i = 0; i < grid_size; ++i)
	    {
		m_Matrix[i] = undirected_graph.m_Matrix[i];
	    }
	}
	else
	{
	    m_Matrix = NULL;
	}
	m_inverse_Matrix = undirected_graph.m_inverse_Matrix;
	
	m_Vertices.clear();
	m_Edges.clear();

	add_Vertices(undirected_graph.m_Vertices.size());
	for (sInt_32 i = 0; i < undirected_graph.m_Vertices.size(); ++i)
	{
	    m_Vertices[i].m_Conflicts = undirected_graph.m_Vertices[i].m_Conflicts;
	    m_Vertices[i].m_capacity = undirected_graph.m_Vertices[i].m_capacity;	    
	}	

	for (Edges_list::const_iterator edge = undirected_graph.m_Edges.begin(); edge != undirected_graph.m_Edges.end(); ++edge)
	{
	    if (edge->m_directed)
	    {
		add_Arrow(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    else
	    {
		add_Edge(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	}	

	m_all_pairs_distances_calculated = undirected_graph.m_all_pairs_distances_calculated;
	m_all_pairs_Distances = undirected_graph.m_all_pairs_Distances;

	m_source_goal_distances_calculated = undirected_graph.m_source_goal_distances_calculated;
	m_source_Distances = undirected_graph.m_source_Distances;
	m_goal_Distances = undirected_graph.m_goal_Distances;

	return *this;
    }


    sUndirectedGraph::~sUndirectedGraph()
    {
	if (m_Matrix != NULL)
	{
	    delete m_Matrix;
	}
    }
    

/*----------------------------------------------------------------------------*/
    
    void sUndirectedGraph::generate_Star(sInt_32 N_vertices)
    {
       	add_Vertices(N_vertices);

	for (sInt_32 u_id = 1; u_id < N_vertices; ++u_id)
	{
	    add_Edge(0, u_id);
	}       	
    }

    
    void sUndirectedGraph::generate_Clique(sInt_32 N_vertices)
    {
       	add_Vertices(N_vertices);

	for (sInt_32 u_id = 0; u_id < N_vertices - 1; ++u_id)
	{
	    for (sInt_32 v_id = u_id + 1; v_id < N_vertices; ++v_id)
	    {
		add_Edge(u_id, v_id);
	    }
	}	
    }
	  

    void sUndirectedGraph::generate_Path(sInt_32 N_vertices)
    {
	add_Vertices(N_vertices);

	for (sInt_32 u_id = 1; u_id < N_vertices; ++u_id)
	{
	    add_Edge(u_id - 1, u_id);
	}       	
    }

    
/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::initialize_InverseMatrix(void)
    {	
	if (m_Matrix != NULL)
	{
	    m_inverse_Matrix.resize(get_VertexCount());
		
	    for (sInt_32 j = 0; j < m_y_size; ++j)
	    {
		for (sInt_32 i = 0; i < m_x_size; ++i)
		{
		    sInt_32 vertex_ID = m_Matrix[j * m_x_size + i];
		    
		    if (vertex_ID > -1)
		    {
			m_inverse_Matrix[vertex_ID].m_row = j;
			m_inverse_Matrix[vertex_ID].m_column = i;
		    }
		}
	    }
	}
    }


    void sUndirectedGraph::set_Capacities(sInt_32 capacity)
    {
	for (sInt_32 u_id = 1; u_id < m_Vertices.size(); ++u_id)
	{
	    m_Vertices[u_id].m_capacity = capacity;
	}	
    }


/*----------------------------------------------------------------------------*/
    
    void sUndirectedGraph::add_Vertex(void)
    {
	m_Vertices.push_back(sVertex(m_Vertices.size()));
    }


    void sUndirectedGraph::add_Vertex(sInt_32 id, sInt_32 capacity)
    {
	sASSERT(m_Vertices.size() == id);
	m_Vertices.push_back(sVertex(m_Vertices.size(), capacity));
    }


    void sUndirectedGraph::add_Vertices(sInt_32 Vertex_cnt)
    {
	while (Vertex_cnt-- > 0)
	{
	    m_Vertices.push_back(sVertex(m_Vertices.size()));
	}
    }


    sInt_32 sUndirectedGraph::get_VertexCount(void) const
    {
	return m_Vertices.size();
    }


    sVertex* sUndirectedGraph::get_Vertex(sInt_32 id)
    {
	return &m_Vertices[id];
    }


    const sVertex* sUndirectedGraph::get_Vertex(sInt_32 id) const
    {
	return &m_Vertices[id];
    }


    bool sUndirectedGraph::is_Grid(void) const
    {
	return (m_Matrix != NULL);
    }


    sInt_32 sUndirectedGraph::get_GridHeight(void) const
    {
	sASSERT(m_Matrix != NULL);
	return m_y_size;
    }

    
    sInt_32 sUndirectedGraph::get_GridWidth(void) const
    {
	sASSERT(m_Matrix != NULL);
	return m_x_size;
    }


    sInt_32 sUndirectedGraph::get_GridCell(sInt_32 x, sInt_32 y) const
    {
	sASSERT(m_Matrix != NULL);
	return m_Matrix[y * m_x_size + x];
    }


    sInt_32 sUndirectedGraph::calc_GridRow(sInt_32 vertex_id) const
    {
	sASSERT(m_Matrix != NULL);

	for (sInt_32 j = 0; j < m_y_size; ++j)
	{
	    for (sInt_32 i = 0; i < m_x_size; ++i)
	    {
		if (m_Matrix[j * m_x_size + i] == vertex_id)
		{
		    return j;
		}
	    }
	}
	sASSERT(false);
	return -1;
    }


    sInt_32 sUndirectedGraph::calc_GridColumn(sInt_32 vertex_id) const
    {
	sASSERT(m_Matrix != NULL);

	for (sInt_32 j = 0; j < m_y_size; ++j)
	{
	    for (sInt_32 i = 0; i < m_x_size; ++i)
	    {
		if (m_Matrix[j * m_x_size + i] == vertex_id)
		{
		    return i;
		}
	    }
	}
	sASSERT(false);
	return -1;
    }


    sInt_32 sUndirectedGraph::calc_GridVertexID(sInt_32 grid_row, sInt_32 grid_column) const
    {
	sASSERT(m_Matrix != NULL);
	return m_Matrix[grid_row * m_x_size + grid_column];
    }


    sInt_32 sUndirectedGraph::calc_GridNeighborVertexID(sInt_32 vertex_id, sInt_32 delta_row, sInt_32 delta_column) const
    {
	sInt_32 row = calc_GridRow(vertex_id);
	sInt_32 column = calc_GridColumn(vertex_id);

	sInt_32 next_row = row + delta_row;
	sInt_32 next_column = column + delta_column;

	if (next_row >= 0 && next_column < m_y_size && next_column >= 0 && next_column < m_x_size)
	{
	    return calc_GridVertexID(next_row, next_column);
	}
	else
	{
	    return -1;
	}
    }


    sInt_32 sUndirectedGraph::get_GridNeighborVertexID(sInt_32 vertex_id, sInt_32 delta_row, sInt_32 delta_column) const
    {
	sASSERT(m_Matrix != NULL);
		
	sInt_32 row = m_inverse_Matrix[vertex_id].m_row;
	sInt_32 column = m_inverse_Matrix[vertex_id].m_column;

	sInt_32 next_row = row + delta_row;
	sInt_32 next_column = column + delta_column;	

	if (next_row >= 0 && next_column < m_y_size && next_column >= 0 && next_column < m_x_size)
	{
	    return calc_GridVertexID(next_row, next_column);
	}
	else
	{
	    return -1;
	}	
    }
    


    sInt_32 sUndirectedGraph::get_EdgeCount(void) const
    {
	return m_Edge_cnt;
	//	return m_Edges.size();
    }


    void sUndirectedGraph::add_Arrow(sInt_32 u_id, sInt_32 v_id)
    {
	sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);
	sASSERT(u_id != v_id);

	sInt_32 id = m_Edge_cnt++;
	m_Edges.push_back(sEdge(id, &m_Vertices[u_id], &m_Vertices[v_id], true));
	sEdge &inserted_edge = m_Edges.back();

	m_Vertices[u_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[v_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[u_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);
    }


    void sUndirectedGraph::add_Edge(sInt_32 u_id, sInt_32 v_id)
    {
	sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);
	sASSERT(u_id != v_id);

	sInt_32 id = m_Edge_cnt++;
	m_Edges.push_back(sEdge(id, &m_Vertices[u_id], &m_Vertices[v_id]));
	sEdge &inserted_edge = m_Edges.back();	

	m_Vertices[u_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[v_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[u_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);
	
	m_Vertices[v_id].m_Neighbors.push_back(&inserted_edge.m_arc_vu);
	m_Vertices[u_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_vu);
	m_Vertices[v_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_vu);
    }


    void sUndirectedGraph::add_RandomArrow(sInt_32 u_id, sInt_32 v_id)
    {
	sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);
	sASSERT(u_id != v_id);

	sInt_32 rnd = rand() % 2;

	if (rnd == 1)
	{
	    sInt_32 id = m_Edge_cnt++;
	    m_Edges.push_back(sEdge(id, &m_Vertices[u_id], &m_Vertices[v_id], true));
	    sEdge &inserted_edge = m_Edges.back();

	    m_Vertices[u_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[v_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[u_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);
	}
	else
	{
	    sInt_32 id = m_Edge_cnt++;
	    m_Edges.push_back(sEdge(id, &m_Vertices[v_id], &m_Vertices[u_id], true));
	    sEdge &inserted_edge = m_Edges.back();

	    m_Vertices[v_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[u_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[v_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);	    
	}
    }    


    bool sUndirectedGraph::is_Adjacent(sInt_32 u_id, sInt_32 v_id) const
    {
	if (m_directed)
	{
	    for (sVertex::Neighbors_list::const_iterator u_neighbor = m_Vertices[u_id].m_Neighbors.begin(); u_neighbor != m_Vertices[u_id].m_Neighbors.end(); ++u_neighbor)
	    {
		if ((*u_neighbor)->m_target->m_id == v_id)
		{
		    return true;
		}
	    }	    
	}
	else
	{
	    sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);

	    if (m_Vertices[u_id].m_Neighbors.size() < m_Vertices[v_id].m_Neighbors.size())
	    {
		for (sVertex::Neighbors_list::const_iterator u_neighbor = m_Vertices[u_id].m_Neighbors.begin(); u_neighbor != m_Vertices[u_id].m_Neighbors.end(); ++u_neighbor)
		{
		    if ((*u_neighbor)->m_target->m_id == v_id)
		    {
			return true;
		    }
		}
	    }
	    else
	    {
		for (sVertex::Neighbors_list::const_iterator v_neighbor = m_Vertices[v_id].m_Neighbors.begin(); v_neighbor != m_Vertices[v_id].m_Neighbors.end(); ++v_neighbor)
		{
		    if ((*v_neighbor)->m_target->m_id == u_id)
		    {
			return true;
		    }
		}
	    }
	}
	return false;
    }


    bool sUndirectedGraph::is_LinkedTo(sInt_32 u_id, sInt_32 v_id) const
    {
	for (sVertex::Neighbors_list::const_iterator u_neighbor = m_Vertices[u_id].m_out_Neighbors.begin(); u_neighbor != m_Vertices[u_id].m_out_Neighbors.end(); ++u_neighbor)
	{
	    if ((*u_neighbor)->m_target->m_id == v_id)
	    {
		return true;
	    }
	}
	return false;
    }


/*----------------------------------------------------------------------------*/
    
    sInt_32 sUndirectedGraph::calc_ShortestPath(sInt_32 u_id, sInt_32 v_id) const
    {
	Distances_vector Distances;
	calc_SingleSourceShortestPathsBreadth(u_id, Distances);

	return Distances[v_id];
    }


    void sUndirectedGraph::calc_SingleSourceShortestPaths(sInt_32 s_id, Distances_vector &Distances) const
    {
	Distances.resize(m_Vertices.size(), sINT_32_MAX);

	Distances[s_id] = 0;
	VertexQueue_multimap vertex_Queue;
	vertex_Queue.insert(VertexQueue_multimap::value_type(0, s_id));

	while (!vertex_Queue.empty())
	{
	    VertexQueue_multimap::value_type front_record = *vertex_Queue.begin();

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_record.second].m_Neighbors.begin(); neighbor != m_Vertices[front_record.second].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;
		sInt_32 distance_update = front_record.first + 1;
		if (Distances[neighbor_id] == sINT_32_MAX || Distances[neighbor_id] > distance_update)
		{
		    Distances[neighbor_id] = distance_update;
		    vertex_Queue.insert(VertexQueue_multimap::value_type(distance_update, neighbor_id));
		}
	    }
	    vertex_Queue.erase(vertex_Queue.begin());
	}
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth(sInt_32 s_id, Distances_vector &Distances) const
    {
	Distances.resize(m_Vertices.size(), sINT_32_MAX);
	Distances[s_id] = 0;

	VertexIDs_vector vertex_Queue_;
	vertex_Queue_.resize(m_Vertices.size(), sINT_32_MAX);
	sInt_32 queue_bottom = 0;
	sInt_32 queue_top = 0;
	vertex_Queue_[queue_top++] = s_id;

	while (queue_top > queue_bottom)
	{
	    sInt_32 front_id = vertex_Queue_[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (Distances[neighbor_id] == sINT_32_MAX)
		{
		    Distances[neighbor_id] = Distances[front_id] + 1;
		    vertex_Queue_[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth_opposite(sInt_32 s_id, Distances_vector &Distances) const
    {
	Distances.resize(m_Vertices.size(), sINT_32_MAX);
	Distances[s_id] = 0;

	VertexIDs_vector vertex_Queue_;
	vertex_Queue_.resize(m_Vertices.size(), sINT_32_MAX);
	sInt_32 queue_bottom = 0;
	sInt_32 queue_top = 0;
	vertex_Queue_[queue_top++] = s_id;

	sUndirectedGraph oppo_graph(*this, true);

	while (queue_top > queue_bottom)
	{
	    sInt_32 front_id = vertex_Queue_[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = oppo_graph.m_Vertices[front_id].m_Neighbors.begin(); neighbor != oppo_graph.m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (Distances[neighbor_id] == sINT_32_MAX)
		{
		    Distances[neighbor_id] = Distances[front_id] + 1;
		    vertex_Queue_[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth_opposite(sInt_32 s_id)
    {
	sInt_32 N_Vertices = m_Vertices.size();
	m_Distances.resize(N_Vertices, sINT_32_MAX);
	m_Queue.resize(m_Vertices.size(), sINT_32_MAX);

	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_Distances[i] = sINT_32_MAX;
	    m_Queue[i] = sINT_32_MAX;
	}

	m_Distances[s_id] = 0;

	sInt_32 queue_bottom = 0;
	sInt_32 queue_top = 0;
	m_Queue[queue_top++] = s_id;

	sUndirectedGraph oppo_graph(*this, true);	

	while (queue_top > queue_bottom)
	{
	    sInt_32 front_id = m_Queue[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = oppo_graph.m_Vertices[front_id].m_Neighbors.begin(); neighbor != oppo_graph.m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (m_Distances[neighbor_id] == sINT_32_MAX)
		{
		    m_Distances[neighbor_id] = m_Distances[front_id] + 1;
		    m_Queue[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}
    }    

    
/*----------------------------------------------------------------------------*/
    
    void sUndirectedGraph::find_ShortestPathBreadth(sInt_32 source_id, sInt_32 dest_id, VertexIDs_vector &shortest_Path)
    {
	Distances_vector Distances;
	VertexIDs_vector vertex_Queue_;

	Distances.resize(m_Vertices.size(), sINT_32_MAX);
	vertex_Queue_.resize(m_Vertices.size(), sINT_32_MAX);

	sInt_32 queue_bottom = 0;
	sInt_32 queue_top = 0;
	vertex_Queue_[queue_top++] = source_id;
	m_Vertices[source_id].m_prev_id = -1;
	Distances[source_id] = 0;

	while (queue_top > queue_bottom)
	{
	    sInt_32 front_id = vertex_Queue_[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (Distances[neighbor_id] == sINT_32_MAX)
		{
		    Distances[neighbor_id] = Distances[front_id] + 1;
		    m_Vertices[neighbor_id].m_prev_id = front_id;
		    vertex_Queue_[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}

	sInt_32 vertex_id = dest_id;

	do
	{
	    shortest_Path.push_back(vertex_id);
	    vertex_id = m_Vertices[vertex_id].m_prev_id;
	} while (vertex_id != -1);
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth(sInt_32 s_id)
    {
	sInt_32 N_Vertices = m_Vertices.size();
	m_Distances.resize(N_Vertices, sINT_32_MAX);
	m_Queue.resize(m_Vertices.size(), sINT_32_MAX);

	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_Distances[i] = sINT_32_MAX;
	    m_Queue[i] = sINT_32_MAX;
	}

	m_Distances[s_id] = 0;

	sInt_32 queue_bottom = 0;
	sInt_32 queue_top = 0;
	m_Queue[queue_top++] = s_id;

	while (queue_top > queue_bottom)
	{
	    sInt_32 front_id = m_Queue[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (m_Distances[neighbor_id] == sINT_32_MAX)
		{
		    m_Distances[neighbor_id] = m_Distances[front_id] + 1;
		    m_Queue[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances)
    {
	all_pairs_Distances.clear();
	sInt_32 N_Vertices = m_Vertices.size();

	for (sInt_32 source_id = 0; source_id < N_Vertices; ++source_id)
	{
	    calc_SingleSourceShortestPathsBreadth(source_id);
	    all_pairs_Distances.push_back(m_Distances);
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	all_pairs_Distances.clear();
	sInt_32 N_Vertices = m_Vertices.size();

	all_pairs_Distances.resize(N_Vertices);
	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    all_pairs_Distances[i].resize(N_Vertices);
	}

	for (VertexIDs_vector::const_iterator source = source_IDs.begin(); source != source_IDs.end(); ++source)
	{
	    calc_SingleSourceShortestPathsBreadth(*source);

	    sInt_32 vertex_id = 0;

	    for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
	    {
		all_pairs_Distances[*source][vertex_id] = *distance;
		all_pairs_Distances[vertex_id][*source] = *distance;
		++vertex_id;
	    }
	}

	for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	{
	    calc_SingleSourceShortestPathsBreadth(*goal);

	    sInt_32 vertex_id = 0;

	    for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
	    {
		all_pairs_Distances[*goal][vertex_id] = *distance;
		all_pairs_Distances[vertex_id][*goal] = *distance;
		++vertex_id;
	    }
	}
    }


    void sUndirectedGraph::calc_SourceGoalShortestPaths(Distances_2d_vector &source_Distances, Distances_2d_vector &goal_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	source_Distances.clear();
	goal_Distances.clear();
	sInt_32 N_Vertices = m_Vertices.size();

	source_Distances.resize(N_Vertices);
	goal_Distances.resize(N_Vertices);

	for (VertexIDs_vector::const_iterator source = source_IDs.begin(); source != source_IDs.end(); ++source)
	{
	    source_Distances[*source].resize(N_Vertices);
	}

	for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	{
	    goal_Distances[*goal].resize(N_Vertices);
	}

	for (VertexIDs_vector::const_iterator source = source_IDs.begin(); source != source_IDs.end(); ++source)
	{
	    calc_SingleSourceShortestPathsBreadth(*source);

	    sInt_32 vertex_id = 0;

	    for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
	    {
		source_Distances[*source][vertex_id] = *distance;
		++vertex_id;
	    }
	}

	if (m_directed)
	{
	    for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	    {
		calc_SingleSourceShortestPathsBreadth_opposite(*goal);

		sInt_32 vertex_id = 0;
		
		for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
		{
		    goal_Distances[*goal][vertex_id] = *distance;
		    ++vertex_id;
		}
	    }	    
	}
	else
	{
	    for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	    {
		calc_SingleSourceShortestPathsBreadth(*goal);

		sInt_32 vertex_id = 0;
		
		for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
		{
		    goal_Distances[*goal][vertex_id] = *distance;
		    ++vertex_id;
		}
	    }
	}
    }


  void sUndirectedGraph::collect_EquidistantVertices(sInt_32 s_id, sInt_32 distance, VertexIDs_vector &equidistant_IDs)
  {
    	sInt_32 N_Vertices = m_Vertices.size();
	m_Distances.resize(N_Vertices, sINT_32_MAX);
	m_Queue.resize(m_Vertices.size(), sINT_32_MAX);

	for (sInt_32 i = 0; i < N_Vertices; ++i)
	{
	    m_Distances[i] = sINT_32_MAX;
	    m_Queue[i] = sINT_32_MAX;
	}

	m_Distances[s_id] = 0;

	sInt_32 queue_bottom = 0;
	sInt_32 queue_top = 0;
	m_Queue[queue_top++] = s_id;

	if (distance == 0)
	  {
	    equidistant_IDs.push_back(s_id);
	  }

	while (queue_top > queue_bottom)
	{
	    sInt_32 front_id = m_Queue[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		sInt_32 neighbor_id = (*neighbor)->m_target->m_id;

		if (m_Distances[neighbor_id] == sINT_32_MAX)
		{
		    m_Distances[neighbor_id] = m_Distances[front_id] + 1;
		    m_Queue[queue_top++] = neighbor_id;

		    if (m_Distances[neighbor_id] == distance)
		    {
			equidistant_IDs.push_back(neighbor_id);
		    }
		}
	    }
	    ++queue_bottom;
	}
	
  }

    
    void sUndirectedGraph::calc_AllPairsShortestPaths(void)
    {
	if (!m_all_pairs_distances_calculated)
	{
	    calc_AllPairsShortestPaths(m_all_pairs_Distances);
	    m_all_pairs_distances_calculated = true;
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	if (!m_all_pairs_distances_calculated)
	{
	    calc_AllPairsShortestPaths(m_all_pairs_Distances, source_IDs, goal_IDs);
	    m_all_pairs_distances_calculated = true;
	}
    }


    void sUndirectedGraph::calc_SourceGoalShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	if (!m_source_goal_distances_calculated)
	{
	    calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	    m_source_goal_distances_calculated = true;
	}
    }


    const sUndirectedGraph::Distances_2d_vector& sUndirectedGraph::get_AllPairsShortestPaths(void) const
    {
	sASSERT(m_all_pairs_distances_calculated);
	return m_all_pairs_Distances;
    }


    const sUndirectedGraph::Distances_2d_vector& sUndirectedGraph::get_SourceShortestPaths(void) const
    {
	sASSERT(m_source_goal_distances_calculated);
	return m_source_Distances;
    }


    const sUndirectedGraph::Distances_2d_vector& sUndirectedGraph::get_GoalShortestPaths(void) const
    {
	sASSERT(m_source_goal_distances_calculated);
	return m_goal_Distances;
    }


/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::find_ShortestPath(sInt_32 u_id, sInt_32 v_id, VertexIDs_list &path)
    {
	for (Vertices_vector::iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->m_distance = INT_MAX;
	    vertex->m_prev_id = -1;
	}
	find_ShortestPathBFS(u_id, v_id);

	sInt_32 path_id = get_Vertex(v_id)->m_id;
	while (path_id != -1)
	{
	    path.push_front(path_id);
	    path_id = get_Vertex(path_id)->m_prev_id;
	}
    }


    void sUndirectedGraph::find_ShortestPathBFS(sInt_32 u_id, sInt_32 v_id)
    {
	Vertices_list queue;

	sVertex *first_vertex = get_Vertex(u_id);
	first_vertex->m_distance = 0;
	first_vertex->m_prev_id = -1;
	queue.push_back(first_vertex);

	while (!queue.empty())
	{
	    sVertex *vertex = queue.front();
	    queue.pop_front();

	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		if ((*neighbor)->m_target->m_distance > vertex->m_distance + 1)
		{
		    (*neighbor)->m_target->m_distance = vertex->m_distance + 1;
		    (*neighbor)->m_target->m_prev_id = vertex->m_id;

		    queue.push_back((*neighbor)->m_target);

		    if ((*neighbor)->m_target->m_id == v_id)
		    {
			return;
		    }
		}
	    }	    
	}
    }


/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sUndirectedGraph::to_Screen_vertices(const sString &indent) const
    {
	to_Stream_vertices(stdout, indent);
    }


    void sUndirectedGraph::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sUndirected graph: (|V|=%ld |E|=%ld) [\n", indent.c_str(), m_Vertices.size(), m_Edges.size());

	fprintf(fw, "%s%sV = {\n", indent.c_str(), s_INDENT.c_str());
	{	   
	    for (Vertices_vector::const_iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	    {
		vertex->to_Stream(fw, indent + s2_INDENT);
	    }
	}
	fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s%sE = {\n", indent.c_str(), s_INDENT.c_str());
	{	   
	    for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	    {
		edge->to_Stream(fw, indent + s2_INDENT);
	    }
	    fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());
	}
	
	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sUndirectedGraph::to_Stream_vertices(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sUndirected graph: (|V|=%ld, |E|=%ld) [\n", indent.c_str(), m_Vertices.size(), m_Edges.size());

	for (Vertices_vector::const_iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->to_Stream(fw, indent + s_INDENT);
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/
    
    sResult sUndirectedGraph::from_File_cpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_cpf(FILE *fr)
    {
	m_Vertices.clear();
	m_Edges.clear();

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
	c = fgetc(fr);

	while (c == '(')
	{
	    add_Vertex();
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id;

		    while (c != '>')
		    {
			fscanf(fr, "%d", &x_id);
			m_Vertices.back().m_Conflicts.push_back(x_id);
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");
	c = fgetc(fr);

	while (c == (int)'{')
	{
	    sInt_32 u_id, v_id;
	    fscanf(fr, "%d,%d", &u_id, &v_id);

	    if (m_directed)
	    {
		add_Arrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id, y_id;

		    while (c != '>')
		    {
			fscanf(fr, "{%d,%d}", &x_id, &y_id);
//			m_Edges.back().m_Conflicts.push_back(sEdge::Conflict(x_id, y_id));
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}		
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_ccpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_ccpf(FILE *fr)
    {
	m_Vertices.clear();
	m_Edges.clear();

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
	c = fgetc(fr);

	while (c == '(')
	{
	    add_Vertex();
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id;

		    while (c != '>')
		    {
			fscanf(fr, "%d", &x_id);
			m_Vertices.back().m_Conflicts.push_back(x_id);
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");
	c = fgetc(fr);

	while (c == (int)'{')
	{
	    sInt_32 u_id, v_id;
	    fscanf(fr, "%d,%d", &u_id, &v_id);

	    if (m_directed)
	    {
		add_Arrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id, y_id;

		    while (c != '>')
		    {
			fscanf(fr, "{%d,%d}", &x_id, &y_id);
//			m_Edges.back().m_Conflicts.push_back(sEdge::Conflict(x_id, y_id));
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}		
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }    


    sResult sUndirectedGraph::from_File_mpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_mpf(FILE *fr)
    {
	m_Vertices.clear();
	m_Edges.clear();

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
	c = fgetc(fr);

	while (c == '(')
	{
	    add_Vertex();
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id;

		    while (c != '>')
		    {
			fscanf(fr, "%d", &x_id);
			m_Vertices.back().m_Conflicts.push_back(x_id);
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");
	c = fgetc(fr);

	while (c == (int)'{')
	{
	    sInt_32 u_id, v_id;
	    fscanf(fr, "%d,%d", &u_id, &v_id);

	    if (m_directed)
	    {
		add_Arrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id, y_id;

		    while (c != '>')
		    {
			fscanf(fr, "{%d,%d}", &x_id, &y_id);
//			m_Edges.back().m_Conflicts.push_back(sEdge::Conflict(x_id, y_id));
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}		
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_cmpf(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_cmpf(FILE *fr)
    {
	m_Vertices.clear();
	m_Edges.clear();

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
	c = fgetc(fr);

	while (c == '(')
	{
	    sInt_32 id, capacity;
	    fscanf(fr, "%d,%d", &id, &capacity);
	    
	    add_Vertex(id, capacity);    

	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id;

		    while (c != '>')
		    {
			fscanf(fr, "%d", &x_id);
			m_Vertices.back().m_Conflicts.push_back(x_id);
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}
	    }
	    c = fgetc(fr);
	}
	fscanf(fr, " =\n");
	c = fgetc(fr);

	while (c == (int)'{')
	{
	    sInt_32 u_id, v_id;
	    fscanf(fr, "%d,%d", &u_id, &v_id);

	    if (m_directed)
	    {
		add_Arrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    sInt_32 x_id, y_id;

		    while (c != '>')
		    {
			fscanf(fr, "{%d,%d}", &x_id, &y_id);
//			m_Edges.back().m_Conflicts.push_back(sEdge::Conflict(x_id, y_id));
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}		
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }        


    sResult sUndirectedGraph::to_File_cpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_cpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sUndirectedGraph::to_Stream_cpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sE =\n", indent.c_str());

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s{%d,%d} (-1)\n", indent.c_str(), edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	}
    }


    sResult sUndirectedGraph::to_File_ccpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_ccpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sUndirectedGraph::to_Stream_ccpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sE =\n", indent.c_str());

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s{%d,%d} (-1)\n", indent.c_str(), edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	}
    }    


    sResult sUndirectedGraph::to_File_mpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_mpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sUndirectedGraph::to_Stream_mpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sE =\n", indent.c_str());

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s{%d,%d}\n", indent.c_str(), edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	}
    }


    sResult sUndirectedGraph::to_File_cmpf(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_cmpf(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sUndirectedGraph::to_Stream_cmpf(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sE =\n", indent.c_str());

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s{%d,%d}\n", indent.c_str(), edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	}
    }        


    sResult sUndirectedGraph::from_File_map(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_map(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_map(FILE *fr)
    {
	char type_ignore[128];
	sInt_32 x_size, y_size;
	fscanf(fr, "type %s\n", type_ignore);
	fscanf(fr, "height %d\n", &y_size);
	fscanf(fr, "width %d\n", &x_size);

	fscanf(fr, "map\n");

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_bgu(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_bgu(FILE *fr)
    {
	char type_ignore[256];
	sInt_32 x_size, y_size, id;
	fscanf(fr, "%d,%s\n", &id, type_ignore);
	fscanf(fr, "Grid:\n");
	fscanf(fr, "%d,%d\n", &y_size, &x_size);

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::to_File_usc(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_usc(fw, indent);	
	fclose(fw);

	return sRESULT_SUCCESS;	
    }

    
    void sUndirectedGraph::to_Stream_usc(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s%d,%d\n", indent.c_str(), m_y_size, m_x_size);

	for (sInt_32 j = 0; j < m_y_size; ++j)
	{
	    fprintf(fw, "%s", indent.c_str());
	    for (sInt_32 i = 0; i < m_x_size; ++i)
	    {
		if (m_Matrix[j * m_x_size + i] >= 0)
		{
		    fprintf(fw, ".");
		}
		else
		{
		    fprintf(fw, "#");
		}
	    }
	    fprintf(fw, "\n");
	}	
    }
    
    
    sResult sUndirectedGraph::from_File_usc(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_usc(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_usc(FILE *fr)
    {
	sInt_32 x_size, y_size;
	fscanf(fr, "%d,%d\n", &y_size, &x_size);

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
	
	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_lusc(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_lusc(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_lusc(FILE *fr)
    {
	sInt_32 x_size, y_size;
	fscanf(fr, "%d,%d\n", &y_size, &x_size);

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = j * x_size + i;
		sInt_32 v_id = (j + 1) * x_size + i;
		sInt_32 w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (sInt_32 j = 0; j < y_size - 1; ++j)
	{
	    sInt_32 u_id = (j + 1) * x_size - 1;
	    sInt_32 v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (sInt_32 i = 0; i < x_size - 1; ++i)
	{
	    sInt_32 u_id = (y_size - 1) * x_size + i;
	    sInt_32 v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
		
	return sRESULT_SUCCESS;
    }        


    sResult sUndirectedGraph::to_File_dibox(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_dibox(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;	
    }

    
    void sUndirectedGraph::to_Stream_dibox(FILE *fw, const sString &sUNUSED(indent)) const
    {
	char name_ignore[] = "anonymous";
	fprintf(fw, "%s\n", name_ignore);

	sInt_32 N_Vertices = m_Vertices.size();
	sInt_32 N_Edges = m_Edges.size();
	
	fprintf(fw, "NUMBER OF VERTICES:%d\n", N_Vertices);
	fprintf(fw, "NUMBER OF EDGES:%d\n", N_Edges);
	fprintf(fw, "LIST OF EDGES:\n");

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    sInt_32 u_id, v_id;

	    u_id = edge->m_arc_uv.m_source->m_id;
	    v_id = edge->m_arc_uv.m_target->m_id;
	    
	    fprintf(fw, "(%d,%d)\n", u_id + 1, v_id + 1);

	}
    }
    

    sResult sUndirectedGraph::from_File_dibox(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_dibox(FILE *fr)
    {
	char name_ignore[256];
	fscanf(fr, "%s\n", name_ignore);

	sInt_32 N_Vertices, N_Edges;
	
	fscanf(fr, "NUMBER OF VERTICES:%d\n", &N_Vertices);
	fscanf(fr, "NUMBER OF EDGES:%d\n", &N_Edges);
	fscanf(fr, "LIST OF EDGES:\n");

	add_Vertices(N_Vertices);
	
	for (sInt_32 i = 0; i < N_Edges; ++i)
	{
	    sInt_32 u_id, v_id;
	    fscanf(fr, "(%d,%d)\n", &u_id, &v_id);

	    add_Arrow(u_id - 1, v_id - 1);
	}

	return sRESULT_SUCCESS;
    }    

    
    sResult sUndirectedGraph::to_File_xml(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_xml(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;	
    }

    
    void sUndirectedGraph::to_Stream_xml(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s<grid>\n", indent.c_str());

	for (sInt_32 row = 0; row < m_y_size; ++row)
	{

	    fprintf(fw, "%s%s<row>", indent.c_str(), s_INDENT.c_str());
	    
	    sInt_32 column = 0;
	    while (true)
	    {
		if (m_Matrix[row * m_x_size + column] >= 0)
		{
		    fprintf(fw, "0");
		}
		else
		{
		    fprintf(fw, "1");
		}
		if (++column < m_x_size)
		{
		    fprintf(fw, " ");
		}
		else
		{
		    break;
		}
	    }
	    fprintf(fw, "</row>\n");
	}	
	fprintf(fw, "%s</grid>\n", indent.c_str());			
    }
    


    sResult sUndirectedGraph::from_File_movi(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_movi(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_movi(FILE *fr)
    {
	char map_type[128];
	sInt_32 x_size, y_size;
	fscanf(fr, "type %s\n", map_type);
	fscanf(fr, "height %d\n", &y_size);
	fscanf(fr, "width %d\n", &x_size);

	fscanf(fr, "map\n");

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	sInt_32 cnt = 0;

	for (sInt_32 j = 0; j < y_size; ++j)
	{
	    for (sInt_32 i = 0; i < x_size; ++i)
	    {
		char ch;
		
		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	if (strcmp(map_type, "octile") == 0)
	{
	    for (sInt_32 j = 0; j < y_size - 1; ++j)
	    {
		for (sInt_32 i = 0; i < x_size - 1; ++i)
		{
		    sInt_32 u_id = j * x_size + i;
		    sInt_32 v_id = (j + 1) * x_size + i;
		    sInt_32 w_id = j * x_size + i + 1;
		    
		    if (m_Matrix[u_id] != -1)
		    {
			if (m_Matrix[v_id] != -1)
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
			}
			if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		    }
		}
	    }
	    for (sInt_32 j = 0; j < y_size - 1; ++j)
	    {
		sInt_32 u_id = (j + 1) * x_size - 1;
		sInt_32 v_id = (j + 2) * x_size - 1;
		
		if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	    for (sInt_32 i = 0; i < x_size - 1; ++i)
	    {
		sInt_32 u_id = (y_size - 1) * x_size + i;
		sInt_32 v_id = (y_size - 1) * x_size + (i + 1);
		
		if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	    
	return sRESULT_SUCCESS;
    }

    
/*----------------------------------------------------------------------------*/

    sResult sUndirectedGraph::to_File_mapR(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_mapR(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sUndirectedGraph::to_Stream_mapR(FILE *fw, const sString &indent) const
    {
	sInt_32 N_vertices = m_Vertices.size();
	sInt_32 N_edges = m_Edges.size();
	
	fprintf(fw, "%sVertices: %d\n", indent.c_str(), N_vertices);	
	fprintf(fw, "%sEdges: %d\n", indent.c_str(), N_edges);

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s{%d,%d}\n", indent.c_str(), edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	}
    }    

    
    sResult sUndirectedGraph::from_File_mapR(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_mapR(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}

	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_mapR(FILE *fr)
    {
	sInt_32 N_vertices;
	sInt_32 N_edges;	

	fscanf(fr, "Vertices: %d\n", &N_vertices);
	fscanf(fr, "Edges: %d\n", &N_edges);

	add_Vertices(N_vertices);

	for (sInt_32 edge_id = 0; edge_id < N_edges; ++edge_id)
	{
	    sInt_32 u_id, v_id;
	    
	    fscanf(fr, "{%d,%d}\n", &u_id, &v_id);
	    add_Edge(u_id, v_id);	    
	}
	return sRESULT_SUCCESS;
    }    



    
/*----------------------------------------------------------------------------*/
// sVectorGraph

    sVectorGraph::sVectorGraph()
    {
	// nothing
    }


    sVectorGraph::sVectorGraph(const sUndirectedGraph &undirected_graph)
    {
	if (undirected_graph.is_Grid())
	{
	    sInt_32 grid_height = undirected_graph.get_GridHeight();
	    sInt_32 grid_width = undirected_graph.get_GridWidth();
	    m_Connections.resize(grid_height);

	    for (sInt_32 i = 0; i < grid_height; ++i)
	    {
		m_Connections[i].resize(grid_width);
		
		for (sInt_32 j = 0; j < grid_width; ++j)
		{
		    m_Connections[i][j] = (undirected_graph.get_GridCell(i, j) >= 0) ? 1 : 0;
		}
	    }

	}
	else
	{
	    sInt_32 N_vertices = undirected_graph.get_VertexCount();
	    m_Connections.resize(N_vertices);

	    for (sInt_32 i = 0; i < N_vertices; ++i)
	    {
		m_Connections[i].resize(N_vertices);
		
		for (sInt_32 j = 0; j < N_vertices; ++j)
		{
		    m_Connections[i][j] = undirected_graph.is_Adjacent(i, j) ? 1 : 0;
		}
	    }
	}
    }


    sVectorGraph::~sVectorGraph()
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    void sVectorGraph::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sVectorGraph::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sVector Graph: (height=%ld width=%ld) [\n", indent.c_str(), m_Connections.size(), m_Connections[0].size());

	sInt_32 N_vertices = m_Connections.size();
	for (sInt_32 i = 0; i < N_vertices; ++i)
	{
	    fprintf(fw, "%s", indent.c_str());
	    for (sInt_32 j = 0; j < N_vertices; ++j)
	    {
		fprintf(fw, "%d ", m_Connections[i][j]);
	    }
	    fprintf(fw, "\n");
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }

/*----------------------------------------------------------------------------*/

} // namespace boOX
