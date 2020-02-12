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
/* cbs.h / 1-224_leibniz                                                      */
/*----------------------------------------------------------------------------*/
//
// Conflict based search implemented in a standard way. A version for MAPF and
// a version for the token swapping have been implemented.
//
/*----------------------------------------------------------------------------*/


#ifndef __CBS_H__
#define __CBS_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "result.h"

#include "common/types.h"
#include "core/graph.h"
#include "core/agent.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sCBSBase

    class sCBSBase
    {
    public:
	typedef std::vector<sInt_32> VertexIDs_vector;
	typedef std::vector<sInt_32> AgentIDs_vector;
	
	typedef std::list<sInt_32> VertexIDs_list;
	typedef std::unordered_set<sInt_32> VertexIDs_uset;
	typedef std::unordered_set<sInt_32> AgentIDs_uset;
	typedef std::unordered_map<sInt_32, VertexIDs_uset> NeighborIDs_umap;
	
	typedef std::vector<VertexIDs_uset> Conflicts_vector;
	typedef std::vector<NeighborIDs_umap> EdgeConflicts_vector;
	typedef std::vector<VertexIDs_vector> AgentPaths_vector;

	typedef std::vector<Conflicts_vector> AgentConflicts_vector;
	typedef std::vector<EdgeConflicts_vector> AgentEdgeConflicts_vector;		

	struct MonoCollision
	{
	    MonoCollision() { /* nothing */ }
	    
	    MonoCollision(sInt_32 cooccupation,
			  sInt_32 agent_id,
			  sInt_32 level,
			  sInt_32 vertex_id)
	    : m_cooccupation(cooccupation)
	    , m_agent_id(agent_id)
	    , m_level(level)
	    , m_vertex_id(vertex_id)
	    { /* nothing */ }
	    
	    sInt_32 m_cooccupation;
	    
	    sInt_32 m_agent_id;
	    sInt_32 m_level;
	    sInt_32 m_vertex_id;
	    
	    bool operator<(const MonoCollision &collision) const
	    {
		return (m_cooccupation < collision.m_cooccupation);
	    }	    
	};

	typedef std::vector<MonoCollision> MonoCollisions_vector;	

	struct CapacitatedCollision
	{
	    CapacitatedCollision() { /* nothing */ }

	    bool operator<(const CapacitatedCollision &sUNUSED(capacitated_collision)) const
	    {
		// TODO
		return true;
	    }	    	    

	    MonoCollisions_vector m_mono_Collisions;
	};

	typedef std::vector<CapacitatedCollision> CapacitatedCollisions_vector;	
	
	struct Collision
	{
	    Collision() { /* nothing */ }
	    
	    Collision(sInt_32 cooccupation,
		      sInt_32 agent_A_id,
		      sInt_32 agent_B_id,
		      sInt_32 level,
		      sInt_32 vertex_id)
	    : m_cooccupation(cooccupation)
	    , m_agent_A_id(agent_A_id)
	    , m_agent_B_id(agent_B_id)
	    , m_level_A(level)
	    , m_level_B(level)		
	    , m_vertex_A_id(vertex_id)
	    , m_vertex_B_id(vertex_id)
	    { /* nothing */ }
	    
	    Collision(sInt_32 cooccupation,
		      sInt_32 agent_A_id,
		      sInt_32 agent_B_id,
		      sInt_32 level_A,
		      sInt_32 level_B,
		      sInt_32 vertex_A_id,
		      sInt_32 vertex_B_id)
	    : m_cooccupation(cooccupation)
	    , m_agent_A_id(agent_A_id)
	    , m_agent_B_id(agent_B_id)
	    , m_level_A(level_A)
	    , m_level_B(level_B)		
	    , m_vertex_A_id(vertex_A_id)
	    , m_vertex_B_id(vertex_B_id)
	    { /* nothing */ }

	    sInt_32 m_cooccupation;
	    
	    sInt_32 m_agent_A_id;
	    sInt_32 m_agent_B_id;
	    sInt_32 m_level_A;
	    sInt_32 m_level_B; 
	    sInt_32 m_vertex_A_id;
	    sInt_32 m_vertex_B_id;	    
	    
	    bool operator<(const Collision &collision) const
	    {
		return (m_cooccupation < collision.m_cooccupation);
	    }	    
	};

	struct EdgeCollision
	{
	    EdgeCollision() { /* nothing */ }
  	    
	    EdgeCollision(sInt_32 cooccupation,
			  sInt_32 agent_A_id,
			  sInt_32 agent_B_id,
			  sInt_32 level,
			  sInt_32 edge_A_u_id,
			  sInt_32 edge_A_v_id,		      
			  sInt_32 edge_B_u_id,
			  sInt_32 edge_B_v_id)
	    : m_cooccupation(cooccupation)
	    , m_agent_A_id(agent_A_id)
	    , m_agent_B_id(agent_B_id)
	    , m_level_A(level)
	    , m_level_B(level)
	    , m_edge_A_u_id(edge_A_u_id)
	    , m_edge_A_v_id(edge_A_v_id)
	    , m_edge_B_u_id(edge_B_u_id)
	    , m_edge_B_v_id(edge_B_v_id)		
	    { /* nothing */ }

	    sInt_32 m_cooccupation;
	    
	    sInt_32 m_agent_A_id;
	    sInt_32 m_agent_B_id;
	    sInt_32 m_level_A;
	    sInt_32 m_level_B;

	    sInt_32 m_edge_A_u_id;
	    sInt_32 m_edge_A_v_id;

	    sInt_32 m_edge_B_u_id;
	    sInt_32 m_edge_B_v_id;	    

	    bool operator<(const EdgeCollision &collision) const
	    {
		return (m_cooccupation < collision.m_cooccupation);
	    }

	    bool operator<(const Collision &collision) const
	    {
		return (m_cooccupation < collision.m_cooccupation);
	    }	    	    
	};	

	typedef std::unordered_map<sInt_32, sInt_32> Occupation_umap;
	typedef std::vector<Occupation_umap> Occupations_vector;

	typedef std::unordered_map<sInt_32, AgentIDs_uset> Cooccupation_umap;
	typedef std::vector<Cooccupation_umap> Cooccupations_vector;

	typedef std::multiset<Collision, std::less<Collision> > Collisions_mset;
	typedef std::multiset<EdgeCollision, std::less<EdgeCollision> > EdgeCollisions_mset;
	/*----------------------------------------------------------------------------*/	

    public:
	sCBSBase(sInstance *instance);
	sCBSBase(sInstance *instance, sDouble timeout);	
	/*----------------------------------------------------------------------------*/

	sInt_32 fill_Cooccupations(const sInstance &instance, const AgentPaths_vector &agent_Paths, Cooccupations_vector &space_Cooccupations) const;		
	void cast_Occupations(sInt_32 agent_id, sInt_32 prefix_length, const VertexIDs_vector &path, Occupations_vector &space_Occupations) const;
	void uncast_Occupations(sInt_32 prefix_length, const VertexIDs_vector &path, Occupations_vector &space_Occupations) const;
	/*----------------------------------------------------------------------------*/	

    private:
	sCBSBase(const sCBSBase &cbs_base);
	const sCBSBase& operator=(const sCBSBase &cbs_base);

    public:
	sInstance *m_Instance;
	sDouble m_timeout;
    };
    

/*----------------------------------------------------------------------------*/
// sCBS

    class sCBS
	: public sCBSBase
    {
    public:
	struct Visit
	{
	    Visit() { /* nothing */ }
	    Visit(sInt_32 level, sInt_32 vertex_id, sInt_32 previous_id)
	    : m_level(level), m_vertex_id(vertex_id), m_previous_id(previous_id) { /* nothing */ }
	    
	    sInt_32 m_level;
	    sInt_32 m_vertex_id;
	    sInt_32 m_previous_id;
	};

	struct Conflict
	{
	    Conflict() { /* nothing */ }
	    Conflict(sInt_32 vertex_id, sInt_32 agent_id, sInt_32 level)
	    : m_vertex_id(vertex_id), m_agent_id(agent_id), m_level(level) { /* nothing */ }

	    sInt_32 m_vertex_id;
	    sInt_32 m_agent_id;
	    sInt_32 m_level;
	};

	struct EdgeConflict
	{
	    EdgeConflict() { /* nothing */ }
	    EdgeConflict(sInt_32 edge_u_id, sInt_32 edge_v_id, sInt_32 agent_id, sInt_32 level)
	    : m_edge_u_id(edge_u_id)
	    , m_edge_v_id(edge_v_id)	    
	    , m_agent_id(agent_id)
	    , m_level(level) { /* nothing */ }

	    sInt_32 m_edge_u_id;
	    sInt_32 m_edge_v_id;	    
	    sInt_32 m_agent_id;
	    sInt_32 m_level;
	};	

	typedef std::vector<Collision> Collisions_vector;
	typedef std::vector<EdgeCollision> EdgeCollisions_vector;	

	struct Node
	{
	    Node(sInt_32 upd_agent_id)
	    : m_upd_agent_id(upd_agent_id)
	    { /* nothing */ }
	    
	    Node(sInt_32 cost, sInt_32 tanglement, sInt_32 upd_agent_id)
	    : m_cost(cost), m_tanglement(tanglement), m_upd_agent_id(upd_agent_id)
	    { /* nothing */ }
	    
	    Node(sInt_32                     upd_agent_id,
		 const AgentConflicts_vector agent_Conflicts,
		 const AgentPaths_vector     &agent_Paths)
	    : m_upd_agent_id(upd_agent_id)
	    , m_agent_Paths(agent_Paths)
	    , m_agent_Conflicts(agent_Conflicts)
	    { /* nothing */ }

	    Node(sInt_32                         upd_agent_id,
		 const AgentConflicts_vector     agent_Conflicts,
		 const AgentEdgeConflicts_vector agent_edge_Conflicts,		 
		 const AgentPaths_vector         &agent_Paths)
	    : m_upd_agent_id(upd_agent_id)
	    , m_agent_Paths(agent_Paths)
	    , m_agent_Conflicts(agent_Conflicts)
	    , m_agent_edge_Conflicts(agent_edge_Conflicts)		
	    { /* nothing */ }

	    Node(sInt_32                  upd_agent_id,
		 const AgentPaths_vector &agent_Paths)
	    : m_upd_agent_id(upd_agent_id)
	    , m_agent_Paths(agent_Paths)
	    { /* nothing */ }	    	    

	    Node(sInt_32                      cost,
		 sInt_32                      tanglement,
		 sInt_32                      upd_agent_id,
		 const AgentConflicts_vector &agent_Conflicts,
		 const AgentPaths_vector     &agent_Paths)
	    : m_cost(cost)
	    , m_tanglement(tanglement)
	    , m_upd_agent_id(upd_agent_id)
	    , m_agent_Paths(agent_Paths)
	    , m_agent_Conflicts(agent_Conflicts)
	    { /* nothing */ }

	    Node(sInt_32                          cost,
		 sInt_32                          tanglement,
		 sInt_32                          upd_agent_id,
		 const AgentConflicts_vector     &agent_Conflicts,
		 const AgentEdgeConflicts_vector &agent_edge_Conflicts,
		 const AgentPaths_vector         &agent_Paths)
	    : m_cost(cost)
	    , m_tanglement(tanglement)
	    , m_upd_agent_id(upd_agent_id)
	    , m_agent_Paths(agent_Paths)
	    , m_agent_Conflicts(agent_Conflicts)
	    , m_agent_edge_Conflicts(agent_edge_Conflicts)		
	    { /* nothing */ }

	    Node(sInt_32                  cost,
		 sInt_32                  tanglement,
		 sInt_32                  upd_agent_id,
		 const AgentPaths_vector &agent_Paths)
	    : m_cost(cost)
	    , m_tanglement(tanglement)
	    , m_upd_agent_id(upd_agent_id)
	    , m_agent_Paths(agent_Paths)
	    { /* nothing */ }

	    bool operator<(const Node &node) const
	    {
		return ((m_cost < node.m_cost) || (m_cost == node.m_cost && m_tanglement < node.m_tanglement));
	    }	    

	    sInt_32 m_node_id;	    

	    sInt_32 m_upper_node_id;
	    sInt_32 m_left_node_id;
	    sInt_32 m_right_node_id;

	    Conflict *m_next_conflict;
	    EdgeConflict *m_next_edge_conflict;
	    VertexIDs_vector *m_next_path;
	    VertexIDs_vector *m_prev_path;

	    sInt_32 m_cost;
	    sInt_32 m_tanglement;	    
	    sInt_32 m_upd_agent_id;
	    
	    AgentPaths_vector m_agent_Paths;
	    AgentConflicts_vector m_agent_Conflicts;
	    AgentEdgeConflicts_vector m_agent_edge_Conflicts;

	    sInt_32 mm_depth;
	};

	typedef std::vector<Node> Nodes_vector;

	struct NodeReference
	{
	    NodeReference(sInt_32 node_id, const Nodes_vector *nodes_Store)
	    : m_node_id(node_id)
	    , m_nodes_Store(nodes_Store)
	    {
		// nothing
	    }

	    bool operator<(const NodeReference &node_ref) const
	    {
		return (((*m_nodes_Store)[m_node_id].m_cost < (*node_ref.m_nodes_Store)[node_ref.m_node_id].m_cost)
			|| ((*m_nodes_Store)[m_node_id].m_cost == (*node_ref.m_nodes_Store)[node_ref.m_node_id].m_cost
			    && (*m_nodes_Store)[m_node_id].m_tanglement < (*node_ref.m_nodes_Store)[node_ref.m_node_id].m_tanglement));
	    }

	    sInt_32 m_node_id;
	    const Nodes_vector *m_nodes_Store;
	};

	typedef std::unordered_map<sInt_32, Visit> Visits_umap;
	typedef std::vector<Visits_umap> Visits_vector;
	typedef std::list<Visit> Visits_list;
	typedef std::multimap<sInt_32, Visit> Visits_mmap;
	
	typedef std::map<sInt_32, Node, std::less<sInt_32> > Nodes_map;
	typedef std::multimap<sInt_32, Node, std::less<sInt_32>> Nodes_mmap;

	typedef std::set<Node, std::less<Node> > Nodes_set;
	typedef std::multiset<Node, std::less<Node> > Nodes_mset;

	typedef std::set<NodeReference, std::less<NodeReference> > NodeReferences_set;
	typedef std::multiset<NodeReference, std::less<NodeReference> > NodeReferences_mset;

	typedef std::vector<sInt_32> NodeIDs_vector;	
	
    public:
	sCBS(sInstance *instance);
	sCBS(sInstance *instance, sDouble timeout);
	/*----------------------------------------------------------------------------*/

	sInt_32 find_ShortestNonconflictingSwapping(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingSwapping(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwapping(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingSwapping(sInstance         &instance,
						    AgentPaths_vector &agent_Paths,
						    sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwapping_Delta(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingSwapping_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);

	sInt_32 find_ShortestNonconflictingSwapping_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingSwapping_Delta(sInstance         &instance,
							 AgentPaths_vector &agent_Paths,
							 sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingSwapping_DeltaStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingSwapping_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);

	sInt_32 find_ShortestNonconflictingSwapping_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingSwapping_DeltaStar(sInstance         &instance,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingSwapping_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingSwapping_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);

	sInt_32 find_ShortestNonconflictingSwapping_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingSwapping_DeltaSuperStar(sInstance         &instance,
								   AgentPaths_vector &agent_Paths,
								   sInt_32            cost_limit);		

	sInt_32 find_ShortestNonconflictingPaths(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPaths(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;	
	
	sInt_32 find_ShortestNonconflictingPaths(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPaths(sInstance         &instance,
						 AgentPaths_vector &agent_Paths,
						 sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPaths_Delta(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPaths_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingPaths_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPaths_Delta(sInstance         &instance,
						       AgentPaths_vector &agent_Paths,
						       sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPaths_DeltaStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPaths_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingPaths_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPaths_DeltaStar(sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPaths_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPaths_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingPaths_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPaths_DeltaSuperStar(sInstance         &instance,
								AgentPaths_vector &agent_Paths,
								sInt_32            cost_limit);			
	
	sInt_32 find_ShortestNonconflictingPermutation(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPermutation(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;	
	
	sInt_32 find_ShortestNonconflictingPermutation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPermutation(sInstance         &instance,
						       AgentPaths_vector &agent_Paths,
						       sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutation_Delta(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPermutation_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingPermutation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPermutation_Delta(sInstance         &instance,
							     AgentPaths_vector &agent_Paths,
							     sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPermutation_DeltaStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPermutation_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingPermutation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPermutation_DeltaStar(sInstance         &instance,
								 AgentPaths_vector &agent_Paths,
								 sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPermutation_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPermutation_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingPermutation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPermutation_DeltaSuperStar(sInstance         &instance,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit);		

	sInt_32 find_ShortestNonconflictingRotation(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingRotation(const sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;	
	
	sInt_32 find_ShortestNonconflictingRotation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingRotation(sInstance         &instance,
						    AgentPaths_vector &agent_Paths,
						    sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotation_Delta(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingRotation_Delta(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingRotation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingRotation_Delta(sInstance         &instance,
							  AgentPaths_vector &agent_Paths,
							  sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingRotation_DeltaStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingRotation_DeltaStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingRotation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingRotation_DeltaStar(sInstance         &instance,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingRotation_DeltaSuperStar(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingRotation_DeltaSuperStar(const sInstance &instance, sSolution &solution, sInt_32 cost_limit);
	
	sInt_32 find_ShortestNonconflictingRotation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingRotation_DeltaSuperStar(sInstance         &instance,
								   AgentPaths_vector &agent_Paths,
								   sInt_32            cost_limit);		

	sInt_32 find_NonconflictingSwapping(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_NonconflictingSwapping(const sInstance   &instance,
					    AgentPaths_vector &agent_Paths,
					    sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingSwapping_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);	
	sInt_32 find_NonconflictingSwapping_Delta(const sInstance   &instance,
						  AgentPaths_vector &agent_Paths,
						  sInt_32            cost_limit);

	sInt_32 find_NonconflictingSwapping_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingSwapping_DeltaStar(sInstance         &instance,
						      AgentPaths_vector &agent_Paths,
						      sInt_32            cost_limit,
						      sInt_32            extra_cost);

	sInt_32 find_NonconflictingSwapping_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingSwapping_DeltaSuperStar(sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit,
							   sInt_32            extra_cost);		

	sInt_32 find_NonconflictingPaths(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_NonconflictingPaths(const sInstance   &instance,
					 AgentPaths_vector &agent_Paths,
					 sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPaths_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_NonconflictingPaths_Delta(const sInstance   &instance,
					       AgentPaths_vector &agent_Paths,
					       sInt_32            cost_limit);
	
	sInt_32 find_NonconflictingPaths_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingPaths_DeltaStar(sInstance         &instance,
						   AgentPaths_vector &agent_Paths,
						   sInt_32            cost_limit,
						   sInt_32            extra_cost);

	sInt_32 find_NonconflictingPaths_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingPaths_DeltaSuperStar(sInstance         &instance,
							AgentPaths_vector &agent_Paths,
							sInt_32            cost_limit,
							sInt_32            extra_cost);	

	sInt_32 find_NonconflictingPermutation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_NonconflictingPermutation(const sInstance   &instance,
					       AgentPaths_vector &agent_Paths,
					       sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPermutation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_NonconflictingPermutation_Delta(const sInstance   &instance,
						     AgentPaths_vector &agent_Paths,
						     sInt_32            cost_limit);

	sInt_32 find_NonconflictingPermutation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingPermutation_DeltaStar(sInstance         &instance,
							 AgentPaths_vector &agent_Paths,
							 sInt_32            cost_limit,
							 sInt_32            extra_cost);

	sInt_32 find_NonconflictingPermutation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingPermutation_DeltaSuperStar(sInstance         &instance,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit,
							      sInt_32            extra_cost);		

	sInt_32 find_NonconflictingRotation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_NonconflictingRotation(const sInstance   &instance,
					    AgentPaths_vector &agent_Paths,
					    sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingRotation_Delta(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);
	sInt_32 find_NonconflictingRotation_Delta(const sInstance   &instance,
						  AgentPaths_vector &agent_Paths,
						  sInt_32            cost_limit);

	sInt_32 find_NonconflictingRotation_DeltaStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingRotation_DeltaStar(sInstance         &instance,
						      AgentPaths_vector &agent_Paths,
						      sInt_32            cost_limit,
						      sInt_32            extra_cost);

	sInt_32 find_NonconflictingRotation_DeltaSuperStar(AgentPaths_vector &agent_Paths, sInt_32 cost_limit, sInt_32 extra_cost);
	sInt_32 find_NonconflictingRotation_DeltaSuperStar(sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit,
							   sInt_32            extra_cost);		
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingSwapping_baseRecompute(const sInstance           &instance,
							  AgentConflicts_vector     &agent_Conflicts,
							  AgentEdgeConflicts_vector &agent_edge_Conflicts,
							  AgentPaths_vector         &agent_Paths,
							  sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingSwapping_pathUpdating(const sInstance           &instance,
							 AgentConflicts_vector     &agent_Conflicts,
							 AgentEdgeConflicts_vector &agent_edge_Conflicts,
							 AgentPaths_vector         &agent_Paths,
							 sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingSwapping_prioritizedQueue(const sInstance           &instance,
							     AgentConflicts_vector     &agent_Conflicts,
							     AgentEdgeConflicts_vector &agent_edge_Conflicts,
							     AgentPaths_vector         &agent_Paths,
							     sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingSwapping_prioritizedTanglement(const sInstance           &instance,
								  AgentConflicts_vector     &agent_Conflicts,
								  AgentEdgeConflicts_vector &agent_edge_Conflicts,
								  AgentPaths_vector         &agent_Paths,
								  sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingSwapping_prioritizedCooccupation(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingSwapping_principalCollision(const sInstance           &instance,
							       AgentConflicts_vector     &agent_Conflicts,
							       AgentEdgeConflicts_vector &agent_edge_Conflicts,
							       AgentPaths_vector         &agent_Paths,
							       sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingSwapping_principalCollision_Delta(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit);

	sInt_32 find_NonconflictingSwapping_principalCollision_DeltaStar(const sInstance           &instance,
									 AgentConflicts_vector     &agent_Conflicts,
									 AgentEdgeConflicts_vector &agent_edge_Conflicts,
									 AgentPaths_vector         &agent_Paths,
									 sInt_32                    cost_limit,
									 sInt_32                    extra_cost);

	sInt_32 find_NonconflictingSwapping_principalCollision_DeltaSuperStar(const sInstance           &instance,
									      AgentConflicts_vector     &agent_Conflicts,
									      AgentEdgeConflicts_vector &agent_edge_Conflicts,
									      AgentPaths_vector         &agent_Paths,
									      sInt_32                    cost_limit,
									      sInt_32                    extra_cost);	
	

	sInt_32 update_NonconflictingSwapping(sInt_32                    upd_agent_id,
					      const sInstance           &instance,
					      Occupations_vector        &space_Occupations,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      AgentPaths_vector         &agent_Paths,
					      sInt_32                    cost_limit) const;
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingPaths_baseRecompute(const sInstance           &instance,
						       AgentConflicts_vector     &agent_Conflicts,
						       AgentEdgeConflicts_vector &agent_edge_Conflicts,
						       AgentPaths_vector         &agent_Paths,
						       sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPaths_pathUpdating(const sInstance           &instance,
						      AgentConflicts_vector     &agent_Conflicts,
						      AgentEdgeConflicts_vector &agent_edge_Conflicts,
						      AgentPaths_vector         &agent_Paths,
						      sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingPaths_prioritizedQueue(const sInstance           &instance,
							  AgentConflicts_vector     &agent_Conflicts,
							  AgentEdgeConflicts_vector &agent_edge_Conflicts,
							  AgentPaths_vector         &agent_Paths,
							  sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingPaths_prioritizedTanglement(const sInstance           &instance,
							       AgentConflicts_vector     &agent_Conflicts,
							       AgentEdgeConflicts_vector &agent_edge_Conflicts,
							       AgentPaths_vector         &agent_Paths,
							       sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPaths_prioritizedCooccupation(const sInstance           &instance,
								 AgentConflicts_vector     &agent_Conflicts,
								 AgentEdgeConflicts_vector &agent_edge_Conflicts,
								 AgentPaths_vector         &agent_Paths,
								 sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPaths_principalCollision(const sInstance           &instance,
							    AgentConflicts_vector     &agent_Conflicts,
							    AgentEdgeConflicts_vector &agent_edge_Conflicts,
							    AgentPaths_vector         &agent_Paths,
							    sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingPaths_principalCollision_Delta(const sInstance           &instance,
								  AgentConflicts_vector     &agent_Conflicts,
								  AgentEdgeConflicts_vector &agent_edge_Conflicts,
								  AgentPaths_vector         &agent_Paths,
								  sInt_32                    cost_limit);

	sInt_32 find_NonconflictingPaths_principalCollision_DeltaStar(const sInstance           &instance,
								      AgentConflicts_vector     &agent_Conflicts,
								      AgentEdgeConflicts_vector &agent_edge_Conflicts,
								      AgentPaths_vector         &agent_Paths,
								      sInt_32                    cost_limit,
								      sInt_32                    extra_cost);

	sInt_32 find_NonconflictingPaths_principalCollision_DeltaSuperStar(const sInstance           &instance,
									   AgentConflicts_vector     &agent_Conflicts,
									   AgentEdgeConflicts_vector &agent_edge_Conflicts,
									   AgentPaths_vector         &agent_Paths,
									   sInt_32                    cost_limit,
									   sInt_32                    extra_cost);	
	
	sInt_32 update_NonconflictingPaths(sInt_32                    upd_agent_id,
					   const sInstance           &instance,
					   Occupations_vector        &space_Occupations,
					   AgentConflicts_vector     &agent_Conflicts,
					   AgentEdgeConflicts_vector &agent_edge_Conflicts,					   
					   AgentPaths_vector         &agent_Paths,
					   sInt_32                    cost_limit) const;	
	/*----------------------------------------------------------------------------*/

	sInt_32 find_NonconflictingPermutation_baseRecompute(const sInstance           &instance,
							     AgentConflicts_vector     &agent_Conflicts,
							     AgentEdgeConflicts_vector &agent_edge_Conflicts,
							     AgentPaths_vector         &agent_Paths,
							     sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPermutation_pathUpdating(const sInstance           &instance,
							    AgentConflicts_vector     &agent_Conflicts,
							    AgentEdgeConflicts_vector &agent_edge_Conflicts,
							    AgentPaths_vector         &agent_Paths,
							    sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPermutation_prioritizedQueue(const sInstance           &instance,
								AgentConflicts_vector     &agent_Conflicts,
								AgentEdgeConflicts_vector &agent_edge_Conflicts,
								AgentPaths_vector         &agent_Paths,
								sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingPermutation_prioritizedTanglement(const sInstance           &instance,
								     AgentConflicts_vector     &agent_Conflicts,
								     AgentEdgeConflicts_vector &agent_edge_Conflicts,								     
								     AgentPaths_vector         &agent_Paths,
								     sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPermutation_prioritizedCooccupation(const sInstance           &instance,
								       AgentConflicts_vector     &agent_Conflicts,
								       AgentEdgeConflicts_vector &agent_edge_Conflicts,
								       AgentPaths_vector         &agent_Paths,
								       sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingPermutation_principalCollision(const sInstance           &instance,
								  AgentConflicts_vector     &agent_Conflicts,
								  AgentEdgeConflicts_vector &agent_edge_Conflicts,
								  AgentPaths_vector         &agent_Paths,
								  sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingPermutation_principalCollision_Delta(const sInstance           &instance,
									AgentConflicts_vector     &agent_Conflicts,
									AgentEdgeConflicts_vector &agent_edge_Conflicts,
									AgentPaths_vector         &agent_Paths,
									sInt_32                    cost_limit);

	sInt_32 find_NonconflictingPermutation_principalCollision_DeltaStar(const sInstance           &instance,
									    AgentConflicts_vector     &agent_Conflicts,
									    AgentEdgeConflicts_vector &agent_edge_Conflicts,
									    AgentPaths_vector         &agent_Paths,
									    sInt_32                    cost_limit,
									    sInt_32                    extra_cost);

	sInt_32 find_NonconflictingPermutation_principalCollision_DeltaSuperStar(const sInstance           &instance,
										 AgentConflicts_vector     &agent_Conflicts,
										 AgentEdgeConflicts_vector &agent_edge_Conflicts,
										 AgentPaths_vector         &agent_Paths,
										 sInt_32                    cost_limit,
										 sInt_32                    extra_cost);		
	
	sInt_32 update_NonconflictingPermutation(sInt_32                    upd_agent_id,
						 const sInstance           &instance,
						 Occupations_vector        &space_Occupations,
						 AgentConflicts_vector     &agent_Conflicts,
						 AgentEdgeConflicts_vector &agent_edge_Conflicts,
						 AgentPaths_vector         &agent_Paths,
						 sInt_32                    cost_limit) const;		
	/*----------------------------------------------------------------------------*/

	sInt_32 find_NonconflictingRotation_baseRecompute(const sInstance           &instance,
							  AgentConflicts_vector     &agent_Conflicts,
							  AgentEdgeConflicts_vector &agent_edge_Conflicts,
							  AgentPaths_vector         &agent_Paths,
        						  sInt_32                    cost_limit) const;
	 
	sInt_32 find_NonconflictingRotation_pathUpdating(const sInstance           &instance,
							 AgentConflicts_vector     &agent_Conflicts,
							 AgentEdgeConflicts_vector &agent_edge_Conflicts,
							 AgentPaths_vector         &agent_Paths,
							 sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingRotation_prioritizedQueue(const sInstance           &instance,
							     AgentConflicts_vector     &agent_Conflicts,
							     AgentEdgeConflicts_vector &agent_edge_Conflicts,
							     AgentPaths_vector         &agent_Paths,
							     sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingRotation_prioritizedTanglement(const sInstance           &instance,
								  AgentConflicts_vector     &agent_Conflicts,
								  AgentEdgeConflicts_vector &agent_edge_Conflicts,
								  AgentPaths_vector         &agent_Paths,
								  sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingRotation_prioritizedCooccupation(const sInstance           &instance,
								    AgentConflicts_vector     &agent_Conflicts,
								    AgentEdgeConflicts_vector &agent_edge_Conflicts,
								    AgentPaths_vector         &agent_Paths,
								    sInt_32                    cost_limit) const;
	
	sInt_32 find_NonconflictingRotation_principalCollision(const sInstance           &instance,
							       AgentConflicts_vector     &agent_Conflicts,
							       AgentEdgeConflicts_vector &agent_edge_Conflicts,
							       AgentPaths_vector         &agent_Paths,
							       sInt_32                    cost_limit) const;

	sInt_32 find_NonconflictingRotation_principalCollision_Delta(const sInstance           &instance,
								     AgentConflicts_vector     &agent_Conflicts,
								     AgentEdgeConflicts_vector &agent_edge_Conflicts,
								     AgentPaths_vector         &agent_Paths,
								     sInt_32                    cost_limit);

	sInt_32 find_NonconflictingRotation_principalCollision_DeltaStar(const sInstance           &instance,
									 AgentConflicts_vector     &agent_Conflicts,
									 AgentEdgeConflicts_vector &agent_edge_Conflicts,
									 AgentPaths_vector         &agent_Paths,
									 sInt_32                    cost_limit,
									 sInt_32                    extra_cost);

	sInt_32 find_NonconflictingRotation_principalCollision_DeltaSuperStar(const sInstance           &instance,
									      AgentConflicts_vector     &agent_Conflicts,
									      AgentEdgeConflicts_vector &agent_edge_Conflicts,
									      AgentPaths_vector         &agent_Paths,
									      sInt_32                    cost_limit,
									      sInt_32                    extra_cost);	
	
	sInt_32 update_NonconflictingRotation(sInt_32                    upd_agent_id,
					      const sInstance           &instance,
					      Occupations_vector        &space_Occupations,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      AgentPaths_vector         &agent_Paths,
					      sInt_32                    cost_limit) const;		
	/*----------------------------------------------------------------------------*/	

	sInt_32 revise_NonconflictingSwapping(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      const AgentPaths_vector   &agent_Paths,
					      sInt_32                    cost_limit,
					      Nodes_mmap                &search_Queue) const;

	sInt_32 revise_NonconflictingSwapping(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,					      
					      const AgentPaths_vector   &agent_Paths,
					      sInt_32                    cost_limit,
					      Nodes_mset                &search_Queue) const;

	sInt_32 revise_NonconflictingSwapping(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      const AgentPaths_vector   &agent_Paths,
					      Cooccupations_vector      &space_Cooccupations,
					      sInt_32                    cost_limit,
					      Nodes_mset                &search_Queue) const;

	sInt_32 examine_NonconflictingSwapping(const sInstance           &instance,
					       AgentConflicts_vector     &agent_Conflicts,
					       AgentEdgeConflicts_vector &agent_edge_Conflicts,
					       const AgentPaths_vector   &agent_Paths,
					       Cooccupations_vector      &space_Cooccupations,
					       sInt_32                    cost_limit,
					       Nodes_mset                &search_Queue) const;

	sInt_32 examine_NonconflictingSwappingDelta(const sInstance           &instance,
						    sInt_32                    upper_node_id,
						    Cooccupations_vector      &space_Cooccupations,
						    sInt_32                    cost_limit,
						    Nodes_vector              &search_Store,
						    NodeReferences_mset       &search_Queue) const;	

	sInt_32 analyze_NonconflictingSwapping(const sInstance           &instance,
					       const AgentPaths_vector   &agent_Paths,
					       sInt_32                   &tanglement) const;
	sInt_32 analyze_NonconflictingSwapping(const sInstance           &instance,
					       AgentConflicts_vector     &agent_Conflicts,
					       AgentEdgeConflicts_vector &agent_edge_Conflicts,
					       const AgentPaths_vector   &agent_Paths,
					       sInt_32                   &tanglement) const;

	sInt_32 analyze_NonconflictingSwapping(const sInstance           &instance,
					       const AgentPaths_vector   &agent_Paths,
					       Cooccupations_vector      &space_Cooccupations,
					       sInt_32                   &tanglement) const;	
	sInt_32 analyze_NonconflictingSwapping(const sInstance           &instance,
					       AgentConflicts_vector     &agent_Conflicts,
					       AgentEdgeConflicts_vector &agent_edge_Conflicts,
					       const AgentPaths_vector   &agent_Paths,
					       Cooccupations_vector      &space_Cooccupations,
					       sInt_32                   &tanglement) const;
	/*----------------------------------------------------------------------------*/
	
	sInt_32 revise_NonconflictingPaths(const sInstance           &instance,
					   AgentConflicts_vector     &agent_Conflicts,
					   AgentEdgeConflicts_vector &agent_edge_Conflicts,
					   const AgentPaths_vector   &agent_Paths,
					   sInt_32                    cost_limit,
					   Nodes_mmap                &search_Queue) const;

	sInt_32 revise_NonconflictingPaths(const sInstance           &instance,
					   AgentConflicts_vector     &agent_Conflicts,
					   AgentEdgeConflicts_vector &agent_edge_Conflicts,
					   const AgentPaths_vector   &agent_Paths,
					   sInt_32                    cost_limit,
					   Nodes_mset                &search_Queue) const;

	sInt_32 revise_NonconflictingPaths(const sInstance           &instance,
					   AgentConflicts_vector     &agent_Conflicts,
					   AgentEdgeConflicts_vector &agent_edge_Conflicts,
					   const AgentPaths_vector   &agent_Paths,
					   Cooccupations_vector      &space_Cooccupations,
					   sInt_32                    cost_limit,
					   Nodes_mset                &search_Queue) const;

	sInt_32 examine_NonconflictingPaths(const sInstance           &instance,
					    AgentConflicts_vector     &agent_Conflicts,
					    AgentEdgeConflicts_vector &agent_edge_Conflicts,					    
					    const AgentPaths_vector   &agent_Paths,
					    Cooccupations_vector      &space_Cooccupations,
					    sInt_32                    cost_limit,
					    Nodes_mset                &search_Queue) const;

	sInt_32 examine_NonconflictingPathsDelta(const sInstance      &instance,
						 sInt_32               upper_node_id,
						 Cooccupations_vector &space_Cooccupations,
						 sInt_32               cost_limit,
						 Nodes_vector         &search_Store,
						 NodeReferences_mset  &search_Queue) const;

	sInt_32 analyze_NonconflictingPaths(const sInstance           &instance,
					    const AgentPaths_vector   &agent_Paths,
					    sInt_32                   &tanglement) const;	
	sInt_32 analyze_NonconflictingPaths(const sInstance           &instance,
					    AgentConflicts_vector     &agent_Conflicts,
					    AgentEdgeConflicts_vector &agent_edge_Conflicts,
					    const AgentPaths_vector   &agent_Paths,
					    sInt_32                   &tanglement) const;

	sInt_32 analyze_NonconflictingPaths(const sInstance           &instance,
					    const AgentPaths_vector   &agent_Paths,
					    Cooccupations_vector      &space_Cooccupations,
					    sInt_32                   &tanglement) const;	
	sInt_32 analyze_NonconflictingPaths(const sInstance           &instance,
					    AgentConflicts_vector     &agent_Conflicts,
					    AgentEdgeConflicts_vector &agent_edge_Conflicts,
					    const AgentPaths_vector   &agent_Paths,
					    Cooccupations_vector      &space_Cooccupations,
					    sInt_32                   &tanglement) const;
	/*----------------------------------------------------------------------------*/
	
	sInt_32 revise_NonconflictingPermutation(const sInstance           &instance,
						 AgentConflicts_vector     &agent_Conflicts,
						 AgentEdgeConflicts_vector &agent_edge_Conflicts,
						 const AgentPaths_vector   &agent_Paths,
						 sInt_32                    cost_limit,
						 Nodes_mmap                &search_Queue) const;

	sInt_32 revise_NonconflictingPermutation(const sInstance           &instance,
						 AgentConflicts_vector     &agent_Conflicts,
						 AgentEdgeConflicts_vector &agent_edge_Conflicts,
						 const AgentPaths_vector   &agent_Paths,
						 sInt_32                    cost_limit,
						 Nodes_mset                &search_Queue) const;

	sInt_32 revise_NonconflictingPermutation(const sInstance           &instance,
						 AgentConflicts_vector     &agent_Conflicts,
						 AgentEdgeConflicts_vector &agent_edge_Conflicts,
						 const AgentPaths_vector   &agent_Paths,
						 Cooccupations_vector      &space_Cooccupations,
						 sInt_32                    cost_limit,
						 Nodes_mset                &search_Queue) const;
	
	sInt_32 examine_NonconflictingPermutation(const sInstance           &instance,
						  AgentConflicts_vector     &agent_Conflicts,
						  AgentEdgeConflicts_vector &agent_edge_Conflicts,
						  const AgentPaths_vector   &agent_Paths,
						  Cooccupations_vector      &space_Cooccupations,
						  sInt_32                    cost_limit,
						  Nodes_mset                &search_Queue) const;

	sInt_32 examine_NonconflictingPermutationDelta(const sInstance      &instance,
						       sInt_32               upper_node_id,
						       Cooccupations_vector &space_Cooccupations,
						       sInt_32               cost_limit,
						       Nodes_vector         &search_Store,
						       NodeReferences_mset  &search_Queue) const;	
	
	sInt_32 analyze_NonconflictingPermutation(const sInstance           &instance,
						  AgentConflicts_vector     &agent_Conflicts,
						  AgentEdgeConflicts_vector &agent_edge_Conflicts,
						  const AgentPaths_vector   &agent_Paths,
						  sInt_32                   &tanglement) const;
	
	sInt_32 analyze_NonconflictingPermutation(const sInstance           &instance,
						  AgentConflicts_vector     &agent_Conflicts,
						  AgentEdgeConflicts_vector &agent_edge_Conflicts,
						  const AgentPaths_vector   &agent_Paths,
						  Cooccupations_vector      &space_Cooccupations,
						  sInt_32                   &tanglement) const;       
	/*----------------------------------------------------------------------------*/
	
	sInt_32 revise_NonconflictingRotation(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      const AgentPaths_vector   &agent_Paths,
					      sInt_32                    cost_limit,
					      Nodes_mmap                &search_Queue) const;
	
	sInt_32 revise_NonconflictingRotation(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      const AgentPaths_vector   &agent_Paths,
					      sInt_32                    cost_limit,
					      Nodes_mset                &search_Queue) const;

	sInt_32 revise_NonconflictingRotation(const sInstance           &instance,
					      AgentConflicts_vector     &agent_Conflicts,
					      AgentEdgeConflicts_vector &agent_edge_Conflicts,
					      const AgentPaths_vector   &agent_Paths,
					      Cooccupations_vector      &space_Cooccupations,
					      sInt_32                    cost_limit,
					      Nodes_mset                &search_Queue) const;
	
	sInt_32 examine_NonconflictingRotation(const sInstance           &instance,
					       AgentConflicts_vector     &agent_Conflicts,
					       AgentEdgeConflicts_vector &agent_edge_Conflicts,
					       const AgentPaths_vector   &agent_Paths,
					       Cooccupations_vector      &space_Cooccupations,
					       sInt_32                    cost_limit,
					       Nodes_mset                &search_Queue) const;

	sInt_32 examine_NonconflictingRotationDelta(const sInstance      &instance,
						    sInt_32               upper_node_id,
						    Cooccupations_vector &space_Cooccupations,
						    sInt_32               cost_limit,
						    Nodes_vector         &search_Store,
						    NodeReferences_mset  &search_Queue) const;	

	sInt_32 analyze_NonconflictingRotation(const sInstance           &instance,
					       AgentConflicts_vector     &agent_Conflicts,
					       AgentEdgeConflicts_vector &agent_edge_Conflicts,
					       const AgentPaths_vector   &agent_Paths,
					       sInt_32                   &tanglement) const;
	
	sInt_32 analyze_NonconflictingRotation(const sInstance           &instance,
					       AgentConflicts_vector     &agent_Conflicts,
					       AgentEdgeConflicts_vector &agent_edge_Conflicts,
					       const AgentPaths_vector   &agent_Paths,
					       Cooccupations_vector      &space_Cooccupations,
					       sInt_32                   &tanglement) const;
	/*----------------------------------------------------------------------------*/	
	sInt_32 find_NonconflictingSequence(const sUndirectedGraph &graph,
					    sInt_32                 source_id,
					    sInt_32                 sink_id,
					    const Conflicts_vector &Conflicts,
					    VertexIDs_vector       &Path) const;

	sInt_32 find_NonconflictingSequence(const sUndirectedGraph     &graph,
					    sInt_32                    source_id,
					    sInt_32                    sink_id,
					    const Conflicts_vector     &Conflicts,
					    const EdgeConflicts_vector &edge_Conflicts,					    
					    VertexIDs_vector           &Path) const;

	sInt_32 findStar_NonconflictingSequence(const sUndirectedGraph     &graph,
						sInt_32                    source_id,
						sInt_32                    sink_id,
						sInt_32                    cost_limit,
						sInt_32                    extra_cost,
						const Conflicts_vector     &Conflicts,
						const EdgeConflicts_vector &edge_Conflicts,					    
						VertexIDs_vector           &Path) const;

	sInt_32 findSuperStar_NonconflictingSequence(const sUndirectedGraph     &graph,
						     sInt_32                    source_id,
						     sInt_32                    sink_id,
						     sInt_32                    cost_limit,
						     sInt_32                    extra_cost,
						     const Conflicts_vector     &Conflicts,
						     const EdgeConflicts_vector &edge_Conflicts,					    
						     VertexIDs_vector           &Path) const;		

	void equalize_NonconflictingSequences(AgentPaths_vector &agent_Paths) const;
	void equalize_NonconflictingSequences(const AgentPaths_vector &agent_Paths, AgentPaths_vector &equal_agent_Paths) const;	
	/*----------------------------------------------------------------------------*/

	void rebuild_NodeConflictsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store);
	void rebuild_InitialNodeConflictsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store);
	void rebuild_SubsequentNodeConflictsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store);

	void rebuild_NodePathsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store);
	void rebuild_InitialNodePathsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store);
	void rebuild_SubsequentNodePathsDelta(sInt_32 agent_id, sInt_32 node_id, const Nodes_vector &search_Store);	
	/*----------------------------------------------------------------------------*/	
	
    private:
	sCBS(const sCBS &cbs);
	const sCBS& operator=(const sCBS &cbs);
	
    public:
	NodeIDs_vector m_delta_conflict_node_IDs;
	AgentConflicts_vector m_delta_agent_Conflicts;
	AgentEdgeConflicts_vector m_delta_agent_edge_Conflicts;

	NodeIDs_vector m_delta_path_node_IDs;
	AgentPaths_vector m_first_agent_Paths;
	AgentPaths_vector m_delta_agent_Paths;

	sUndirectedGraph::Distances_2d_vector m_source_Distances;
	sUndirectedGraph::Distances_2d_vector m_goal_Distances;	
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __CBS_H__ */
