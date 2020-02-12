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
/* agent.h / 1-224_leibniz                                                    */
/*----------------------------------------------------------------------------*/
//
// Agent and multi-agent problem related structures.
//
/*----------------------------------------------------------------------------*/


#ifndef __AGENT_H__
#define __AGENT_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>

#include "result.h"

#include "common/types.h"
#include "core/graph.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sConfiguration

    class sConfiguration
    {
    public:
	static const sInt_32 VACANT_VERTEX = 0; /* Agents are numbered starting with 1 */
	static const sInt_32 UNDEFINED_LOCATION = -1;

	static const sInt_32 RANDOM_WALK_LENGTH = s__DEFAULT_RANDOM_WALK_LENGTH;

    public:
	typedef std::vector<sInt_32> Agents_vector;
	typedef std::vector<sInt_32> Vertices_vector;
	typedef std::vector<sInt_32> VertexIDs_vector;

	typedef std::vector<sInt_32> AgentSizes_vector;	

    public:
	sConfiguration();
	sConfiguration(sInt_32 N_Vertices, sInt_32 N_Agents, bool random = false);
	sConfiguration(const sConfiguration &start_configuration, sInt_32 N_Vertices, sInt_32 N_Agents, bool random = false);
	sConfiguration(const sConfiguration &start_configuration, sInt_32 N_Vertices, sInt_32 N_Agents, sInt_32 N_fixed, bool random = false);
        /*----------------------------------------------------------------------------*/ 	
	
	bool operator==(const sConfiguration &agent_configuration) const;
	bool operator<(const sConfiguration &agent_configuration) const;

	sInt_32 get_AgentCount(void) const;
	sInt_32 get_VertexCount(void) const;

	sInt_32 get_AgentLocation(sInt_32 agent_id) const;
	sInt_32 get_VertexOccupancy(sInt_32 vertex_id) const;

	void place_Agent(sInt_32 agent_id, sInt_32 vertex_id);
	void remove_Agent(sInt_32 agent_id);
	void clean_Vertex(sInt_32 vertex_id);

	void move_Agent(sInt_32 agent_id, sInt_32 dest_vertex_id);
	bool verify_Move(sInt_32 agent_id, sInt_32 dest_vertex_id) const;
	bool check_Move(sInt_32 agent_id, sInt_32 dest_vertex_id) const;
	bool verify_Move(sInt_32 agent_id, sInt_32 dest_vertex_id, const sUndirectedGraph &graph) const;
        /*----------------------------------------------------------------------------*/	

	void randomize(void);

	void generate_Walk(const sConfiguration &start_configuration, const sUndirectedGraph &environment);
	void generate_NovelWalk(const sConfiguration &start_configuration, const sUndirectedGraph &environment);

	sResult generate_Nonconflicting(sInt_32 N_Vertices, sInt_32 N_Agents, const sUndirectedGraph &environment);
	void generate_NovelNonconflictingWalk(const sConfiguration &start_configuration, const sUndirectedGraph &environment);	
	
	void generate_DisjointWalk(const sConfiguration &start_configuration, const sUndirectedGraph &environment);		
	void generate_Walk(const sConfiguration &start_configuration, const sUndirectedGraph &environment, sInt_32 N_fixed);

	void generate_Equidistant(const sConfiguration &start_configuration, sUndirectedGraph &environment, sInt_32 distance);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual void to_Screen_brief(const sString &indent = "") const;
	virtual void to_Stream_brief(FILE *fw, const sString &indent = "") const;	

	virtual sResult to_File_cpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_cpf(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_cpf(const sString &filename, sInt_32 component = 0);
	virtual sResult from_Stream_cpf(FILE *fr, sInt_32 component = 0);

	virtual sResult to_File_ccpf(const sString &filename, const sUndirectedGraph &environment, const sString &indent = "") const;
	virtual void to_Stream_ccpf(FILE *fw, const sUndirectedGraph &environment, const sString &indent = "") const;

	virtual sResult from_File_ccpf(const sString &filename, sUndirectedGraph &environment, sInt_32 component = 0);
	virtual sResult from_Stream_ccpf(FILE *fr, sUndirectedGraph &environment, sInt_32 component = 0);	

	virtual sResult to_File_mpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mpf(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_mpf(const sString &filename, sInt_32 component = 0);
	virtual sResult from_Stream_mpf(FILE *fr, sInt_32 component = 0);

	virtual sResult to_File_cmpf(const sString &filename, const sUndirectedGraph &environment, const sString &indent = "") const;
	virtual void to_Stream_cmpf(FILE *fw, const sUndirectedGraph &environment, const sString &indent = "") const;

	virtual sResult from_File_cmpf(const sString &filename, sUndirectedGraph &environment, sInt_32 component = 0);
	virtual sResult from_Stream_cmpf(FILE *fr, sUndirectedGraph &environment, sInt_32 component = 0);		

    public:
	Agents_vector m_agent_Locs;
	Vertices_vector m_vertex_Occups;

	AgentSizes_vector m_agent_Sizes;
    };


/*----------------------------------------------------------------------------*/
// sInstance

    class sInstance
    {
    public:
	typedef std::vector<sUndirectedGraph> Environments_vector;
	typedef std::vector<sConfiguration> Configurations_vector;

	typedef std::vector<sInt_32> VertexIDs_vector;
	typedef std::set<sInt_32> VertexIDs_set;

	typedef std::vector<VertexIDs_vector> AgentMDD_vector;
	typedef std::vector<AgentMDD_vector> MDD_vector;
	typedef std::vector<VertexIDs_set> AgentMDD_set;

/*
	typedef std::unordered_map<int, int> Indices_map;
	typedef std::vector<Indices_map> AgentMDDIndices_vector;
	typedef std::vector<AgentMDDIndices_vector> MDDIndices_vector;
*/
	typedef std::multimap<sInt_32, sInt_32> AgentIndices_mmap;

	typedef std::unordered_map<sInt_32, sInt_32> InverseVertexIDs_umap;
	typedef std::vector<InverseVertexIDs_umap> InverseAgentMDD_vector;	
	typedef std::vector<InverseAgentMDD_vector> InverseMDD_vector;

    public:
	sInstance();

	sInstance(const sUndirectedGraph &environment,
		  const sConfiguration   &start_configuration,
		  const sConfiguration   &goal_configuration);

    public:
	void collect_Endpoints(VertexIDs_vector &source_IDs, VertexIDs_vector &goal_IDs) const;
	
	sInt_32 estimate_TotalPathCost(sInt_32 &max_individual_cost);
	sInt_32 estimate_TotalSwappingCost(sInt_32 &max_individual_cost);
	sInt_32 estimate_TotalPermutationCost(sInt_32 &max_individual_cost);
	sInt_32 estimate_TotalRotationCost(sInt_32 &max_individual_cost);	
	
	sInt_32 construct_PathMDD(sInt_32     max_total_cost,
				  MDD_vector &MDD,
				  sInt_32    &extra_cost,
				  MDD_vector &extra_MDD);
	
	sInt_32 construct_GraphPathMDD(sUndirectedGraph &graph,
				       sInt_32           max_total_cost,
				       MDD_vector       &MDD,
				       sInt_32          &extra_cost,
				       MDD_vector       &extra_MDD);

	sInt_32 construct_SwappingMDD(sInt_32     max_total_cost,
				      MDD_vector &MDD,
				      sInt_32    &extra_cost,
				      MDD_vector &extra_MDD);
	
	sInt_32 construct_GraphSwappingMDD(sUndirectedGraph &graph,
					   sInt_32           max_total_cost,
					   MDD_vector       &MDD,
					   sInt_32          &extra_cost,
					   MDD_vector       &extra_MDD);

	sInt_32 construct_PermutationMDD(sInt_32     max_total_cost,
					 MDD_vector &MDD,
					 sInt_32    &extra_cost,
					 MDD_vector &extra_MDD);
	
	sInt_32 construct_GraphPermutationMDD(sUndirectedGraph &graph,
					      sInt_32           max_total_cost,
					      MDD_vector       &MDD,
					      sInt_32          &extra_cost,
					      MDD_vector       &extra_MDD);

	sInt_32 construct_RotationMDD(sInt_32     max_total_cost,
				      MDD_vector &MDD,
				      sInt_32    &extra_cost,
				      MDD_vector &extra_MDD);
	
	sInt_32 construct_GraphRotationMDD(sUndirectedGraph &graph,
					   sInt_32           max_total_cost,
					   MDD_vector       &MDD,
					   sInt_32          &extra_cost,
					   MDD_vector       &extra_MDD);
        /*----------------------------------------------------------------------------*/	

	void construct_InverseMDD(const MDD_vector &MDD, InverseMDD_vector &inverse_MDD) const;	
        /*----------------------------------------------------------------------------*/

	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Screen_cpf(const sString &indent = "") const;
	virtual void to_Screen_ccpf(const sString &indent = "") const;	
	virtual void to_Screen_mpf(const sString &indent = "") const;
	virtual void to_Screen_cmpf(const sString &indent = "") const;
	
	virtual void to_Screen_domainPDDL(const sString &indent = "") const;
	virtual void to_Screen_problemPDDL(const sString &indent = "") const;
	virtual void to_Screen_bgu(const sString &indent = "", sInt_32 instance_id = -1) const;
	
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_cpf(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_cpf(FILE *fw, sInt_32 N_agents, const sString &indent = "") const;		
	virtual void to_Stream_ccpf(FILE *fw, const sString &indent = "") const;	
	virtual void to_Stream_mpf(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_mpf(FILE *fw, sInt_32 N_agents, const sString &indent = "") const;	
	virtual void to_Stream_cmpf(FILE *fw, const sString &indent = "") const;
	
	virtual void to_Stream_domainPDDL(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_problemPDDL(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_bgu(FILE *fw, const sString &indent = "", sInt_32 instance_id = -1) const;
	virtual void to_Stream_bgu(FILE *fw, sInt_32 N_agents, const sString &indent = "", sInt_32 instance_id = -1) const;	

	virtual sResult to_File(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_cpf(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_cpf(const sString &filename, sInt_32 N_agents, const sString &indent = "") const;		
	virtual sResult to_File_ccpf(const sString &filename, const sString &indent = "") const;	
	virtual sResult to_File_mpf(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_mpf(const sString &filename, sInt_32 N_agents, const sString &indent = "") const;	
	virtual sResult to_File_cmpf(const sString &filename, const sString &indent = "") const;
	
	virtual sResult to_File_domainPDDL(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_problemPDDL(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_bgu(const sString &filename, const sString &indent = "", sInt_32 instance_id = 0) const;
	virtual sResult to_File_bgu(const sString &filename, sInt_32 N_agents, const sString &indent = "", sInt_32 instance_id = 0) const;	

	virtual sResult from_File_cpf(const sString &filename);
	virtual sResult from_Stream_cpf(FILE *fr);
	
	virtual sResult from_File_ccpf(const sString &filename);
	virtual sResult from_Stream_ccpf(FILE *fr);
	
	virtual sResult from_File_mpf(const sString &filename);
	virtual sResult from_Stream_mpf(FILE *fr);
	
	virtual sResult from_File_cmpf(const sString &filename);
	virtual sResult from_Stream_cmpf(FILE *fr);			       

	virtual sResult from_File_bgu(const sString &filename);	
	virtual sResult from_Stream_bgu(FILE *fr);	

	virtual sResult to_File_usc(const sString &map_filename, const sString &agents_filename) const;
	virtual void to_Stream_usc(FILE *map_fr, FILE *agents_fr, const sString &indent = "") const;
       
	virtual sResult from_File_usc(const sString &map_filename, const sString &agents_filename);
	virtual sResult from_Stream_usc(FILE *map_fr, FILE *agents_fr);

	virtual sResult from_File_lusc(const sString &map_filename, const sString &agents_filename);
	virtual sResult from_Stream_lusc(FILE *map_fr, FILE *agents_fr); 

	virtual sResult to_File_dibox(const sString &filename) const;
	virtual sResult to_Stream_dibox(FILE *fr) const;
	
	virtual sResult from_File_dibox(const sString &filename);
	virtual sResult from_Stream_dibox(FILE *fr);

	virtual sResult from_File_movi(const sString &filename, const sUndirectedGraph &environment, sInt_32 N_agents = -1);
	virtual sResult from_Stream_movi(FILE *fr, const sUndirectedGraph &environment, sInt_32 N_agents = -1);

    public:
	sUndirectedGraph m_environment;
	sConfiguration m_start_configuration;
	sConfiguration m_goal_configuration;	
    };


/*----------------------------------------------------------------------------*/
// sSolution

    class sSolution
    {
    public:
	static const sInt_32 N_STEPS_UNDEFINED = -1;

    public:
	struct Move
	{
	    Move(sInt_32 agent_id, sInt_32 src_vrtx_id, sInt_32 dest_vrtx_id);
	    Move(sInt_32 agent_id, sInt_32 src_vrtx_id, sInt_32 dest_vrtx_id, sInt_32 crt_time);

	    bool is_Undefined(void) const;
	    bool is_Dependent(const Move &move) const;

	    sInt_32 m_agent_id;
	    sInt_32 m_src_vrtx_id;
	    sInt_32 m_dest_vrtx_id;

	    sInt_32 m_crt_time;
	};

	static const Move UNDEFINED_MOVE;

	typedef std::list<Move> Moves_list;

	struct Step
	{
	    Step(sInt_32 time);

	    sInt_32 m_time;
	    Moves_list m_Moves;
	};

	typedef std::vector<Step> Steps_vector;
	
    public:
	sSolution();
	sSolution(sInt_32 start_step, const sSolution &sub_solution);
	sSolution(const sSolution &sub_solution_1, const sSolution sub_solution_2);
	
	bool is_Null(void) const;

	sInt_32 get_MoveCount(void) const;
	sInt_32 get_StepCount(void) const;

	void add_Move(sInt_32 time, const Move &move);

	sInt_32 calc_EmptySteps(void) const;
	void remove_EmptySteps(void);

	sSolution extract_Subsolution(sInt_32 start_step, sInt_32 final_step) const;

	void execute_Solution(const sConfiguration &start_configuration,
			      sConfiguration       &final_configuration,
			      sInt_32               N_Steps = N_STEPS_UNDEFINED) const;

	void execute_Step(const sConfiguration &current_configuration,
			  sConfiguration       &final_configuration,
			  sInt_32               step) const;

	void execute_Solution(sConfiguration &configuration, sInt_32 N_Steps = N_STEPS_UNDEFINED) const;
	void execute_Step(sConfiguration &final_configuration, sInt_32 step) const;

	bool verify_Step(const sConfiguration &configuration, sInt_32 step) const;
	bool check_Step(const sConfiguration &configuration, sInt_32 step) const;

	void filter_Solution(const sConfiguration &start_configuration,
			     const sConfiguration &goal_configuration,
			     sSolution            &filter_solution) const;

	sInt_32 calc_CriticalTimes(void);
	void criticalize_Solution(sSolution &critical_solution);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual void to_Screen(const sUndirectedGraph &grid, const sString &indent = "") const;
	virtual void to_Stream(const sUndirectedGraph &grid, FILE *fw, const sString &indent = "") const;
	virtual sResult to_File(const sUndirectedGraph &grid, const sString &filename, const sString &indent = "") const;
	
	virtual sResult to_File_cpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_cpf(FILE *fw, const sString &indent = "") const;
	virtual sResult to_File_ccpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_ccpf(FILE *fw, const sString &indent = "") const;	
	virtual sResult to_File_mpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mpf(FILE *fw, const sString &indent = "") const;
	virtual sResult to_File_cmpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_cmpf(FILE *fw, const sString &indent = "") const;		

	virtual sResult to_File_graphrec(const sString &filename, const sInstance &instance, const sString &indent = "") const;
	virtual void to_Stream_graphrec(FILE *fw, const sInstance &instance, const sString &indent = "") const;	

	virtual sResult from_File_cpf(const sString &filename);
	virtual sResult from_Stream_cpf(FILE *fr);
	virtual sResult from_File_ccpf(const sString &filename);
	virtual sResult from_Stream_ccpf(FILE *fr);	
	virtual sResult from_File_mpf(const sString &filename);
	virtual sResult from_Stream_mpf(FILE *fr);
	virtual sResult from_File_cmpf(const sString &filename);
	virtual sResult from_Stream_cmpf(FILE *fr);		

    public:
	sInt_32 m_Moves_cnt;
	Steps_vector m_Steps;

	double m_optimality_ratio;
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __AGENT_H__ */
