/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-161                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                                                                            */
/*          pavel.surynek@fit.cvut.cz | <pavel.surynek@fit.cvut.cz>           */
/*        http://users.fit.cvut.cz/surynek | <http://www.surynek.com>         */
/*                                                                            */
/*============================================================================*/
/* agentR.h / 0_iskra-161                                                     */
/*----------------------------------------------------------------------------*/
//
// Repsesentation of continuous and semi-continuous MAPF instance (MAPF-R).
//
/*----------------------------------------------------------------------------*/


#ifndef __AGENT_R_H__
#define __AGENT_R_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>

#include "types.h"
#include "result.h"

#include "core/graph.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{




/*----------------------------------------------------------------------------*/
// sRealConfiguration

    class sRealConfiguration
    {
    public:
	static const sInt_32 VACANT_VERTEX = 0; /* Agents are numbered starting with 1 */
	static const sInt_32 UNDEFINED_LOCATION = -1;

	static const sInt_32 RANDOM_WALK_LENGTH = s__DEFAULT_RANDOM_WALK_LENGTH;

    public:
	sRealConfiguration();

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_mpfR(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_mpfR(const sString &filename, sInt_32 component = 0);
	virtual sResult from_Stream_mpfR(FILE *fr, sInt_32 component = 0);	

    public:
	Agents_vector m_agent_Locs;
	Vertices_vector m_vertex_Occups;

	AgentSizes_vector m_agent_Sizes;
    };


/*----------------------------------------------------------------------------*/
// sRealInstance

    class sRealInstance
    {
    public:
/*
	typedef std::unordered_map<int, int> Indices_map;
	typedef std::vector<Indices_map> AgentMDDIndices_vector;
	typedef std::vector<AgentMDDIndices_vector> MDDIndices_vector;

	typedef std::multimap<sInt_32, sInt_32> AgentIndices_mmap;

	typedef std::unordered_map<sInt_32, sInt_32> InverseVertexIDs_umap;
	typedef std::vector<InverseVertexIDs_umap> InverseAgentMDD_vector;	
	typedef std::vector<InverseAgentMDD_vector> InverseMDD_vector;
*/

    public:
	sRealInstance();

    public:
        /*----------------------------------------------------------------------------*/

	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	
	virtual void to_Screen_mpfR(const sString &indent = "") const;		
	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;	

	virtual sResult to_File(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_mpfR(const sString &filename, const sString &indent = "") const;	

    public:
	sRealConfiguration m_start_configuration;
	sRealConfiguration m_goal_configuration;	
    };


/*----------------------------------------------------------------------------*/
// sRealSolution

    class sRealSolution
    {
    public:
	static const sInt_32 N_STEPS_UNDEFINED = -1;
	
    public:
	sRealSolution();
	
	bool is_Null(void) const;

	sInt_32 get_MoveCount(void) const;
	sInt_32 get_StepCount(void) const;

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	
	virtual void to_Screen_mpfR(const sString &indent = "") const;		
	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;	

	virtual sResult to_File(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_mpfR(const sString &filename, const sString &indent = "") const;
	
	virtual sResult from_File_mpfR(const sString &filename);
	virtual sResult from_Stream_mpfR(FILE *fr);	

    public:
	sInt_32 m_Moves_cnt;
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __AGENT_R_H__ */
