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
/* kruhoR.cpp / 1-224_leibniz                                                 */
/*----------------------------------------------------------------------------*/
//
// Repsesentation of continuous and semi-continuous MAPF instance (MAPF-R).
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
#include "core/kruhoR.h"
#include "util/io.h"
#include "util/statistics.h"


using namespace std;
using namespace boOX;


/*----------------------------------------------------------------------------*/


namespace boOX
{



    
/*----------------------------------------------------------------------------*/
// sKruhobot

    const sDouble sKruhobot::UNIT_RADIUS = 1.0;
    const sDouble sKruhobot::UNIT_VELOCITY = 1.0;
    const sDouble sKruhobot::UNIT_ACCELERATION = 1.0;
    
/*----------------------------------------------------------------------------*/
    
    sKruhobot::sKruhobot()	
    {
	// nothing
    }

    
    sKruhobot::sKruhobot(sInt_32 id, const Properties &properties)
	: m_id(id)
	, m_properties(properties)
    {
	// nothing	
    }

    
    sKruhobot::sKruhobot(sInt_32 id, const Properties &properties, const State &state)
	: m_id(id)
	, m_properties(properties)
	, m_state(state)
    {
	// nothing
    }

    
/*----------------------------------------------------------------------------*/

    const sKruhobot::State& sKruhobot::get_State(void) const
    {
	return m_state;
    }

    
    void sKruhobot::set_State(const State &state)
    {
	m_state = state;
    }


/*----------------------------------------------------------------------------*/
    
    void sKruhobot::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void sKruhobot::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sKruhobot: (id = %d) [\n", indent.c_str(), m_id);
	fprintf(fw, "%s%sproperties: (radius = %.3f, lv = %.3f, la = %.3f, av = %.3f, aa = %.3f, wf = %.3f)\n",
		indent.c_str(),
		s_INDENT.c_str(),
		m_properties.m_radius,
		m_properties.m_linear_velo,
		m_properties.m_linear_accel,
		m_properties.m_angular_velo,
		m_properties.m_angular_accel,
		m_properties.m_wait_factor);

	fprintf(fw, "%s%sstate: (orient = %.3f) [\n", indent.c_str(), s_INDENT.c_str(), m_state.m_orientation);

	fprintf(fw, "%s%sposition: (x = %.3f, y = %.3f)\n",
		indent.c_str(),
		s2_INDENT.c_str(),
		m_state.m_position.m_x,
		m_state.m_position.m_y);
	
	fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());
	
	fprintf(fw, "%s]\n", indent.c_str());		
    }


    void sKruhobot::to_Stream_mpfR(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s%d: ", indent.c_str(), m_id);
	
	fprintf(fw, "[r = %f, lv = %.3f, la = %.3f, av = %.3f, aa = %.3f, wf = %.3f]\n",
		m_properties.m_radius,
		m_properties.m_linear_velo,
		m_properties.m_linear_accel,
		m_properties.m_angular_velo,
		m_properties.m_angular_accel,
		m_properties.m_wait_factor);       
    }

    
    sResult sKruhobot::from_Stream_mpfR(FILE *fr)
    {
	fscanf(fr, "%d: ", &m_id);
	
	fscanf(fr, "[r = %lf, lv = %lf, la = %lf, av = %lf, aa = %lf, wf = %lf]\n",
	       &m_properties.m_radius,
	       &m_properties.m_linear_velo,
	       &m_properties.m_linear_accel,
	       &m_properties.m_angular_velo,
	       &m_properties.m_angular_accel,
	       &m_properties.m_wait_factor);

	return sRESULT_SUCCESS;
    }


	    
    
/*----------------------------------------------------------------------------*/
// sRealConjunction

    sRealConjunction::sRealConjunction(s2DMap *Map)
	: m_Map(Map)
    {
	// nothing
    }


    sRealConjunction::sRealConjunction(s2DMap *Map, sInt_32 N_Kruhobots)
	: m_Map(Map)
    {
	m_kruhobot_Locations.resize(N_Kruhobots + 1);

	for (LocationIDs_vector::iterator location = m_kruhobot_Locations.begin(); location != m_kruhobot_Locations.end(); ++location)
	{
	    *location = UNDEFINED_LOCATION;
	}
    }

    
/*----------------------------------------------------------------------------*/

    sInt_32 sRealConjunction::get_KruhobotCount(void) const
    {
	sASSERT(m_kruhobot_Locations.size() > 0);
	return m_kruhobot_Locations.size() - 1;
    }

    
    void sRealConjunction::place_Kruhobot(sInt_32 kruhobot_id, sInt_32 location_id)
    {
	sASSERT(kruhobot_id > 0 && kruhobot_id < m_kruhobot_Locations.size() && location_id < m_Map->m_Locations.size());	
	m_kruhobot_Locations[kruhobot_id] = location_id;
    }

    
    void sRealConjunction::remove_Kruhobot(sInt_32 kruhobot_id)
    {
	sASSERT(kruhobot_id > 0 && kruhobot_id < m_kruhobot_Locations.size());		
	m_kruhobot_Locations[kruhobot_id] = UNDEFINED_LOCATION;
    }

    
 /*----------------------------------------------------------------------------*/
    
    void sRealConjunction::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void sRealConjunction::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sKruhobot conjunction: (|K| = %ld, |M| = %ld) [\n", indent.c_str(), m_kruhobot_Locations.size() - 1, m_Map->m_Locations.size());

	fprintf(fw, "%s%skruhobot locations: {", indent.c_str(), s_INDENT.c_str());
	
	sInt_32 N_Kruhobots_1 = m_kruhobot_Locations.size();
	for (sInt_32 i = 1; i < N_Kruhobots_1; ++i)
	{
	    fprintf(fw, "%d->%d ", i, m_kruhobot_Locations[i]);
	}
	fprintf(fw, "}\n");	
	
	fprintf(fw, "%s]\n", indent.c_str());
    }
    
    
    void sRealConjunction::to_Stream_mpfR(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sAssignments: %ld\n", indent.c_str(), m_kruhobot_Locations.size() - 1);

	sInt_32 N_Kruhobots_1 = m_kruhobot_Locations.size();	
	for (sInt_32 i = 1; i < N_Kruhobots_1; ++i)
	{
	    fprintf(fw, "%s%s%d->%d\n", indent.c_str(), s_INDENT.c_str(), i, m_kruhobot_Locations[i]);
	}	
    }
    

    sResult sRealConjunction::from_Stream_mpfR(FILE *fr)
    {
	sInt_32 N_kruhobot_Locations;	
	fscanf(fr, "Assignments: %d\n", &N_kruhobot_Locations);
	sInt_32 N_kruhobot_Locations_1 = N_kruhobot_Locations + 1;
	m_kruhobot_Locations.resize(N_kruhobot_Locations_1);

	for (sInt_32 i = 1; i < N_kruhobot_Locations_1; ++i)
	{
	    sInt_32 kruhobot_id;
	    fscanf(fr, "%d->%d\n", &kruhobot_id, &m_kruhobot_Locations[i]);
	    sASSERT(kruhobot_id == i);
	}	
	
	return sRESULT_SUCCESS;
    }


    sResult sRealConjunction::from_File_xml_init(const sString &filename, sInt_32 N_kruhobots)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sREAL_CONJUNCTION_XML_OPEN_ERROR;
	}
	
	if (sFAILED(result = from_Stream_xml_init(fr, N_kruhobots)))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;		
    }

    
    sResult sRealConjunction::from_Stream_xml_init(FILE *fr, sInt_32 N_kruhobots)
    {
	sString root_keyword;
	
	sConsumeUntilChar(fr, '<');
	sConsumeUntilChar(fr, '>');	
	
	sConsumeUntilChar(fr, '<');
	sConsumeAlphaString(fr, root_keyword);
	sConsumeUntilChar(fr, '>');

//	printf("keyword:%s\n", root_keyword.c_str());
	if (root_keyword != "root")
	{
	    return sREAL_CONJUNCTION_UNRECOGNIZED_XML_FORMATTING_ERROR;
	}
	if (m_kruhobot_Locations.empty())
	{
	    m_kruhobot_Locations.push_back(0);
	}
	sInt_32 kruhobot_id = 1;
	
	while(true)
	{	    
	    sString next_keyword;

	    if (N_kruhobots >= 0)
	    {
		if (kruhobot_id > N_kruhobots)
		{
		    break;
		}
	    }	    
	    sConsumeUntilChar(fr, '<');	    
	    sConsumeAlphaString(fr, next_keyword);    
//	    printf("nxt:%s\n", next_keyword.c_str());

	    if (next_keyword == "agent")
	    {
		sString start_keyword;
		sConsumeWhiteSpaces(fr);
		sConsumeAlphaString(fr, start_keyword);

//		printf("start:%s\n", start_keyword.c_str());
		
		if (start_keyword == "start_id")
		{
		    sString init_id_keyword;
		    sInt_32 init_id;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_id_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("ID:%s\n", init_id_keyword.c_str());

		    init_id = sInt_32_from_String(init_id_keyword);
		    if (m_kruhobot_Locations.size() <= kruhobot_id)
		    {
			m_kruhobot_Locations.push_back(init_id);
		    }
		    else
		    {
			m_kruhobot_Locations[kruhobot_id] = init_id;
		    }
		    sConsumeUntilChar(fr, '>');
		}
		else if (start_keyword == "start_i")
		{
		    sString init_y_keyword;
		    sInt_32 init_y;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_y_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDy:%s\n", init_y_keyword.c_str());
		    init_y = sInt_32_from_String(init_y_keyword);		    

		    sString init_x_keyword;
		    sInt_32 init_x;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_x_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDx:%s\n", init_x_keyword.c_str());
		    init_x = sInt_32_from_String(init_x_keyword);
		    
		    sConsumeUntilChar(fr, '>');

		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_y * m_Map->m_Network.m_x_size + init_x];
//		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_x * m_Map->m_Network.m_y_size + init_y];
//		    printf("init_idi 1 (%d,%d):%d\n", m_Map->m_Network.m_x_size, m_Map->m_Network.m_y_size, init_id);		    

		    if (m_kruhobot_Locations.size() <= kruhobot_id)
		    {
			m_kruhobot_Locations.push_back(init_id);
		    }
		    else
		    {
			m_kruhobot_Locations[kruhobot_id] = init_id;
		    }
//		    printf("xy: %d, %d [%d]\n", init_x, init_y, init_id);
//		    printf("SSSize:%ld\n", m_kruhobot_Locations.size());
		}
		else if (start_keyword == "id")
		{
		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');		    
	    			    
		    sString init_y_keyword;
		    sInt_32 init_y;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_y_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDy:%s\n", init_y_keyword.c_str());
		    init_y = sInt_32_from_String(init_y_keyword);		    

		    sString init_x_keyword;
		    sInt_32 init_x;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_x_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDx:%s\n", init_x_keyword.c_str());
		    init_x = sInt_32_from_String(init_x_keyword);
		    
		    sConsumeUntilChar(fr, '>');

		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_y * m_Map->m_Network.m_x_size + init_x];
//		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_x * m_Map->m_Network.m_y_size + init_y];
//		    printf("init_idi 2 (%d,%d):%d\n", m_Map->m_Network.m_x_size, m_Map->m_Network.m_y_size, init_id);

		    if (m_kruhobot_Locations.size() <= kruhobot_id)
		    {
			m_kruhobot_Locations.push_back(init_id);
		    }
		    else
		    {
			m_kruhobot_Locations[kruhobot_id] = init_id;
		    }		    
		}
		else
		{
		    return sREAL_CONJUNCTION_UNRECOGNIZED_XML_FORMATTING_ERROR;		    
		}
		++kruhobot_id;
	    }
	    else
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }

    
    sResult sRealConjunction::from_File_xml_goal(const sString &filename, sInt_32 N_kruhobots)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sREAL_CONJUNCTION_XML_OPEN_ERROR;
	}
	
	if (sFAILED(result = from_Stream_xml_goal(fr, N_kruhobots)))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;		
    }

    
    sResult sRealConjunction::from_Stream_xml_goal(FILE *fr, sInt_32 N_kruhobots)
    {
	sString root_keyword;
	
	sConsumeUntilChar(fr, '<');
	sConsumeUntilChar(fr, '>');	
	
	sConsumeUntilChar(fr, '<');
	sConsumeAlphaString(fr, root_keyword);
	sConsumeUntilChar(fr, '>');

//	printf("keyword:%s\n", root_keyword.c_str());
	if (root_keyword != "root")
	{
	    return sREAL_CONJUNCTION_UNRECOGNIZED_XML_FORMATTING_ERROR;
	}
	if (m_kruhobot_Locations.empty())
	{
	    m_kruhobot_Locations.push_back(0);
	}
	sInt_32 kruhobot_id = 1;
	
	while(true)
	{
	    sString next_keyword;

	    if (N_kruhobots >= 0)
	    {
		if (kruhobot_id > N_kruhobots)
		{
		    break;
		}
	    }
	    sConsumeUntilChar(fr, '<');
	    sConsumeAlphaString(fr, next_keyword);    
//	    printf("nxt:%s\n", next_keyword.c_str());

	    if (next_keyword == "agent")
	    {
		sString start_keyword;
		sConsumeWhiteSpaces(fr);
		sConsumeAlphaString(fr, start_keyword);

//		printf("start:%s\n", start_keyword.c_str());
		
		if (start_keyword == "start_id")
		{
		    sString init_id_keyword;
		    sInt_32 init_id;

		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_id_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("ID:%s\n", init_id_keyword.c_str());

		    init_id = sInt_32_from_String(init_id_keyword);
		    if (m_kruhobot_Locations.size() <= kruhobot_id)
		    {
			m_kruhobot_Locations.push_back(init_id);
		    }
		    else
		    {
			m_kruhobot_Locations[kruhobot_id] = init_id;
		    }
		    sConsumeUntilChar(fr, '>');
		}
		else if (start_keyword == "start_i")
		{
		    sString init_y_keyword;
		    sInt_32 init_y;

		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');

		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');		    
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_y_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDy:%s\n", init_y_keyword.c_str());
		    init_y = sInt_32_from_String(init_y_keyword);		    

		    sString init_x_keyword;
		    sInt_32 init_x;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_x_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDx:%s\n", init_x_keyword.c_str());
		    init_x = sInt_32_from_String(init_x_keyword);
		    
		    sConsumeUntilChar(fr, '>');

		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_y * m_Map->m_Network.m_x_size + init_x];
//		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_x * m_Map->m_Network.m_y_size + init_y];
//		    printf("init_id 1:%d\n", init_id);		    

		    if (m_kruhobot_Locations.size() <= kruhobot_id)
		    {
			m_kruhobot_Locations.push_back(init_id);
		    }
		    else
		    {
			m_kruhobot_Locations[kruhobot_id] = init_id;
		    }
//		    printf("xy: %d, %d [%d]\n", init_x, init_y, init_id);
//		    printf("SSSize:%ld\n", m_kruhobot_Locations.size());
		}
		else if (start_keyword == "id")
		{    
		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');		    

		    sString init_y_keyword;
		    sInt_32 init_y;

		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');

		    sConsumeUntilChar(fr, '"');
		    sConsumeUntilChar(fr, '"');		    
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_y_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDy:%s\n", init_y_keyword.c_str());
		    init_y = sInt_32_from_String(init_y_keyword);		    

		    sString init_x_keyword;
		    sInt_32 init_x;
				
		    sConsumeUntilChar(fr, '"');
		    sConsumeAlnumString(fr, init_x_keyword);
		    sConsumeUntilChar(fr, '"');
//		    printf("IDx:%s\n", init_x_keyword.c_str());
		    init_x = sInt_32_from_String(init_x_keyword);
		    
		    sConsumeUntilChar(fr, '>');

		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_y * m_Map->m_Network.m_x_size + init_x];
//		    sInt_32 init_id = m_Map->m_Network.m_Matrix[init_x * m_Map->m_Network.m_y_size + init_y];
//		    printf("textu:%d,%d\n", init_x, init_y);
//		    printf("init_id 2 (%d,%d):%d\n", m_Map->m_Network.m_x_size, m_Map->m_Network.m_y_size, init_id);

		    if (m_kruhobot_Locations.size() <= kruhobot_id)
		    {
			m_kruhobot_Locations.push_back(init_id);
		    }
		    else
		    {
			m_kruhobot_Locations[kruhobot_id] = init_id;
		    }
//		    printf("xy: %d, %d [%d]\n", init_x, init_y, init_id);
//		    printf("SSSize:%ld\n", m_kruhobot_Locations.size());		    
		}
		else
		{
		    return sREAL_CONJUNCTION_UNRECOGNIZED_XML_FORMATTING_ERROR;		    
		}
		++kruhobot_id;
	    }
	    else
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }

    
/*----------------------------------------------------------------------------*/
// sRealInstance
    
    sRealInstance::sRealInstance(s2DMap *Map)
	: m_start_conjunction(Map)
	, m_goal_conjunction(Map)
    {
	m_Kruhobots.push_back(sKruhobot());
    }

    
    sRealInstance::sRealInstance(const sRealConjunction &start_conjunction, const sRealConjunction &goal_conjunction)
	: m_start_conjunction(start_conjunction)
	, m_goal_conjunction(goal_conjunction)
    {
	m_Kruhobots.push_back(sKruhobot());
    }


/*----------------------------------------------------------------------------*/

    void sRealInstance::add_Kruhobot(const sKruhobot &kruhobot)
    {
	m_Kruhobots.push_back(kruhobot);
	m_Kruhobots.back().m_id = m_Kruhobots.size() - 1;
    }

    
    void sRealInstance::add_Kruhobot(sInt_32 id, const sKruhobot &kruhobot)
    {
	if (m_Kruhobots.size() >= id)	    
	{
	    m_Kruhobots.resize(id + 1);
	}
	m_Kruhobots[id] = kruhobot;
	m_Kruhobots[id].m_id = id;
    }
    

/*----------------------------------------------------------------------------*/

    void sRealInstance::collect_StartingLocations(LocationIDs_vector &start_location_IDs) const
    {
	sInt_32 N_locations = m_start_conjunction.m_Map->m_Locations.size();
	vector<sInt_32> collected;

	collected.resize(N_locations, -1);
	sInt_32 N_kruhobots = m_start_conjunction.get_KruhobotCount();
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 start_loc_id = m_start_conjunction.m_kruhobot_Locations[kruhobot_id];
	    if (collected[start_loc_id] < 0)
	    {
		start_location_IDs.push_back(start_loc_id);
		collected[start_loc_id] = 1;
	    }
	}
    }

    
    void sRealInstance::collect_GoalLocations(LocationIDs_vector &goal_location_IDs) const
    {
	sInt_32 N_locations = m_goal_conjunction.m_Map->m_Locations.size();
	vector<sInt_32> collected;

	collected.resize(N_locations, -1);
	sInt_32 N_kruhobots = m_goal_conjunction.get_KruhobotCount();
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 goal_loc_id = m_goal_conjunction.m_kruhobot_Locations[kruhobot_id];
	    if (collected[goal_loc_id] < 0)
	    {
		goal_location_IDs.push_back(goal_loc_id);
		collected[goal_loc_id] = 1;
	    }
	}
    }

    
    void sRealInstance::collect_StartingGoalLocations(LocationIDs_vector &start_goal_location_IDs) const
    {
	sInt_32 N_locations = m_start_conjunction.m_Map->m_Locations.size();
	sASSERT(N_locations == m_goal_conjunction.m_Map->m_Locations.size());
	vector<sInt_32> collected;

	collected.resize(N_locations, -1);
	sInt_32 N_kruhobots = m_goal_conjunction.get_KruhobotCount();	

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 start_loc_id = m_start_conjunction.m_kruhobot_Locations[kruhobot_id];
	    if (collected[start_loc_id] < 0)
	    {
		start_goal_location_IDs.push_back(start_loc_id);
		collected[start_loc_id] = 1;
	    }
	}
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 goal_loc_id = m_goal_conjunction.m_kruhobot_Locations[kruhobot_id];
	    if (collected[goal_loc_id] < 0)
	    {
		start_goal_location_IDs.push_back(goal_loc_id);
		collected[goal_loc_id] = 1;
	    }
	}	
    }
    

/*----------------------------------------------------------------------------*/
    
    void sRealInstance::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }
    
    
    void sRealInstance::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sReal instance [\n", indent.c_str());

	fprintf(fw, "%s%sstart_conjunction = (\n", indent.c_str(), s_INDENT.c_str());
	m_start_conjunction.to_Stream(fw, indent + s2_INDENT);
	fprintf(fw, "%s%s)\n", indent.c_str(), s_INDENT.c_str());

	fprintf(fw, "%s%sgoal_conjunction = (\n", indent.c_str(), s_INDENT.c_str());
	m_goal_conjunction.to_Stream(fw, indent + s2_INDENT);
	fprintf(fw, "%s%s)\n", indent.c_str(), s_INDENT.c_str());	

	fprintf(fw, "%s%skruhobots = {\n", indent.c_str(), s_INDENT.c_str());

	sInt_32 N_Kruhobots_1 = m_Kruhobots.size();
	for (sInt_32 i = 1; i < N_Kruhobots_1; ++i)
	{
	    m_Kruhobots[i].to_Stream(fw, indent + s2_INDENT);
	}	
	fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());
	
	fprintf(fw, "%s]\n", indent.c_str());	
    }


    sResult sRealInstance::to_File_mpfR(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sREAL_INSTANCE_OPEN_ERROR;
	}
	
	to_Stream_mpfR(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;	
    }

    
    void sRealInstance::to_Stream_mpfR(FILE *fw, const sString &indent) const
    {
	sInt_32 N_Kruhobots_1 = m_Kruhobots.size();		
	fprintf(fw, "%sKruhobots: %d\n", indent.c_str(), N_Kruhobots_1 - 1);

	for (sInt_32 i = 1; i < N_Kruhobots_1; ++i)
	{
	    m_Kruhobots[i].to_Stream_mpfR(fw, indent + s_INDENT);
	}
	fprintf(fw, "%sStart\n", indent.c_str());
	m_start_conjunction.to_Stream_mpfR(fw, indent + s_INDENT);
	fprintf(fw, "%sGoal\n", indent.c_str());
	m_goal_conjunction.to_Stream_mpfR(fw, indent + s_INDENT);	
    }

    
    sResult sRealInstance::from_File_mpfR(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sREAL_INSTANCE_OPEN_ERROR;
	}
	
	if (sFAILED(result = from_Stream_mpfR(fr)))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }

    
    sResult sRealInstance::from_Stream_mpfR(FILE *fr)
    {
	sResult result;
	
	sInt_32 N_Kruhobots;		
	fscanf(fr, "Kruhobots: %d\n", &N_Kruhobots);
	sInt_32 N_Kruhobots_1 = N_Kruhobots + 1;	
	m_Kruhobots.resize(N_Kruhobots_1);
	
	for (sInt_32 i = 1; i < N_Kruhobots_1; ++i)
	{
	    if (sFAILED(result = m_Kruhobots[i].from_Stream_mpfR(fr)))
	    {
		return result;
	    }
	}
	
	fscanf(fr, "Start\n");
	if (sFAILED(result = m_start_conjunction.from_Stream_mpfR(fr)))
	{
	    return result;
	}
	fscanf(fr, "Goal\n");
	if (sFAILED(result = m_goal_conjunction.from_Stream_mpfR(fr)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/
    
    sResult sRealInstance::from_File_movi(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sREAL_INSTANCE_MOVISCEN_OPEN_ERROR;
	}
	
	if (sFAILED(result = from_Stream_movi(fr)))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;	
    }

    
    sResult sRealInstance::from_Stream_movi(FILE *fr)
    {
	sInt_32 version_unused;		
	fscanf(fr, "version %d\n", &version_unused);

	sInt_32 N_Kruhobots_1 = 1;

	while (!feof(fr))
	{
	    sInt_32 number_ignore;
	    sChar map_name_ignore[128];

	    sInt_32 x_size, y_size;

	    sInt_32 x_start, y_start;
	    sInt_32 x_goal, y_goal;

	    sDouble real_number_ignore;
	    
	    fscanf(fr, "%d %s %d %d %d %d %d %d %lf\n", &number_ignore, map_name_ignore, &x_size, &y_size, &x_start, &y_start, &x_goal, &y_goal, &real_number_ignore);
//	    printf("%d %s %d %d %d %d %d %d %.3f\n", number_ignore, map_name_ignore, x_size, y_size, x_start, y_start, x_goal, y_goal, real_number_ignore);

	    sInt_32 start_location_id = m_start_conjunction.m_Map->m_Network.m_Matrix[y_start * x_size + x_start];
//	    printf("Map0 point: %d\n", start_location_id);
	    m_start_conjunction.m_kruhobot_Locations.push_back(start_location_id);

	    sInt_32 goal_location_id = m_goal_conjunction.m_Map->m_Network.m_Matrix[y_goal * x_size + x_goal];
//	    printf("Map+ point: %d\n", goal_location_id);
	    m_goal_conjunction.m_kruhobot_Locations.push_back(goal_location_id);	    
	    	    
	    ++N_Kruhobots_1;
	}
	m_Kruhobots.resize(N_Kruhobots_1);
	
	return sRESULT_SUCCESS;
    }

  
    sResult sRealInstance::to_File_xml(const sString &filename, sInt_32 N_kruhobots, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sREAL_INSTANCE_OPEN_ERROR;
	}
	
	to_Stream_xml(fw, N_kruhobots, indent);
	fclose(fw);

	return sRESULT_SUCCESS;	
    }

    
    void sRealInstance::to_Stream_xml(FILE *fw, sInt_32 N_kruhobots, const sString &indent) const
    {
	fprintf(fw, "%s<?xml version=\"1.0\" ?>\n", indent.c_str());
	fprintf(fw, "%s<root>\n", indent.c_str());
	sASSERT(m_start_conjunction.m_kruhobot_Locations.size() == m_goal_conjunction.m_kruhobot_Locations.size());	

	LocationIDs_vector::const_iterator start_location = m_start_conjunction.m_kruhobot_Locations.begin();
	LocationIDs_vector::const_iterator goal_location = m_goal_conjunction.m_kruhobot_Locations.begin();	


	if (N_kruhobots > 0)
	{
	    while (   start_location != m_start_conjunction.m_kruhobot_Locations.end()
		      && goal_location != m_goal_conjunction.m_kruhobot_Locations.end())
	    {
		if (N_kruhobots-- <= 0)
		{
		    break;
		}
		sInt_32 start_location_id = *start_location++;
		sInt_32 goal_location_id = *goal_location++;
		
//	    printf("START: %.3f, %.3f\n", m_start_conjunction.m_Map->m_Locations[start_location_id].m_x, m_start_conjunction.m_Map->m_Locations[start_location_id].m_y);
//	    printf("GOAL: %.3f, %.3f\n", m_goal_conjunction.m_Map->m_Locations[goal_location_id].m_x, m_goal_conjunction.m_Map->m_Locations[goal_location_id].m_y);
		sInt_32 start_i = m_start_conjunction.m_Map->m_Locations[start_location_id].m_y;
		sInt_32 start_j = m_start_conjunction.m_Map->m_Locations[start_location_id].m_x;
		
		sInt_32 goal_i = m_goal_conjunction.m_Map->m_Locations[goal_location_id].m_y;
		sInt_32 goal_j = m_goal_conjunction.m_Map->m_Locations[goal_location_id].m_x;
		
		fprintf(fw, "%s%s<agent start_i=\"%d\" start_j=\"%d\" goal_i=\"%d\" goal_j=\"%d\"/>\n", indent.c_str(), s_INDENT.c_str(), start_i, start_j, goal_i, goal_j);	    
	    }
	}
	else
	{
	    while (   start_location != m_start_conjunction.m_kruhobot_Locations.end()
		      && goal_location != m_goal_conjunction.m_kruhobot_Locations.end())
	    {
		sInt_32 start_location_id = *start_location++;
		sInt_32 goal_location_id = *goal_location++;
		
		sInt_32 start_i = m_start_conjunction.m_Map->m_Locations[start_location_id].m_y;
		sInt_32 start_j = m_start_conjunction.m_Map->m_Locations[start_location_id].m_x;
		
		sInt_32 goal_i = m_goal_conjunction.m_Map->m_Locations[goal_location_id].m_y;
		sInt_32 goal_j = m_goal_conjunction.m_Map->m_Locations[goal_location_id].m_x;

		fprintf(fw, "%s%s<agent start_i=\"%d\" start_j=\"%d\" goal_i=\"%d\" goal_j=\"%d\"/>\n", indent.c_str(), s_INDENT.c_str(), start_i, start_j, goal_i, goal_j);
	    }
	}
	fprintf(fw, "%s</root>\n", indent.c_str());	
    }
        

/*----------------------------------------------------------------------------*/
// sRealSolution

    const sRealSolution::Motion sRealSolution::UNDEFINED_MOTION = sRealSolution::Motion(-1, -1, -1, 0.0, 0.0);

    
/*----------------------------------------------------------------------------*/
    
    sRealSolution::sRealSolution()
    {
	// nothing
    }
    
	
    bool sRealSolution::is_Null(void) const
    {
	return (m_Motions.empty());
    }

    
    sInt_32 sRealSolution::get_MotionCount(void) const
    {
	return (m_Motions.size());
    }
    
    
/*----------------------------------------------------------------------------*/
    
    void sRealSolution::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void sRealSolution::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sSemi-continuous agent solution: (|motions| = %ld) [\n", indent.c_str(), m_Motions.size());

	for (Motions_set::const_iterator motion = m_Motions.begin(); motion != m_Motions.end(); ++motion)
	{
	    fprintf(fw, "%s%s%d: %d --> %d <%.3f,%.3f>\n",
		    indent.c_str(),
		    s_INDENT.c_str(),
		    motion->m_kruhobot_id,
		    motion->m_src_loc_id,
		    motion->m_dest_loc_id,
		    motion->m_duration.m_start_time,
		    motion->m_duration.m_finish_time);	    
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }



    
/*----------------------------------------------------------------------------*/

} // namespace boOX
