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
/* kruhoR.h / 1-224_leibniz                                                   */
/*----------------------------------------------------------------------------*/
//
// Repsesentation of continuous and semi-continuous MAPF instance (MAPF-R).
//
/*----------------------------------------------------------------------------*/


#ifndef __KRUHO_R_H__
#define __KRUHO_R_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>

#include "result.h"

#include "common/types.h"
#include "core/mapR.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sKruhobot

    class sKruhobot
    {
    public:
	static const sDouble UNIT_RADIUS;
	static const sDouble UNIT_VELOCITY;
	static const sDouble UNIT_ACCELERATION;

	struct Position
	{
	    Position() { /* nothing */ }	    
	    Position(sDouble x, sDouble y)
	    : m_x(x)
	    , m_y(y) { /* nothing */ }
	    
	    sDouble m_x, m_y;
	};

	struct State
	{
	    State() { /* nothing */ }
	    State(sDouble orientation, const Position &position)
	    : m_orientation(orientation)
	    , m_position(position) { /* nothing */ }
		
	    sDouble m_orientation; /* (0, 2 * PI) */
	    Position m_position;
	};

	struct Properties
	{
	    Properties() { /* nothing */ }
	    Properties(sDouble radius,
		       sDouble linear_velo,
		       sDouble linear_accel,		    
		       sDouble angular_velo,
		       sDouble angular_accel,
		       sDouble wait_factor)
	    : m_radius(radius)
	    , m_linear_velo(linear_velo)
	    , m_linear_accel(linear_accel)
	    , m_angular_velo(angular_velo)
	    , m_angular_accel(angular_accel)
	    , m_wait_factor(wait_factor)
	    { /* nothing */}

	    sDouble m_radius;
	    sDouble m_linear_velo;
	    sDouble m_linear_accel;
	    sDouble m_angular_velo;
	    sDouble m_angular_accel;
	    sDouble m_wait_factor;
	};
	
    public:
	sKruhobot();
	sKruhobot(sInt_32 id, const Properties &properties);
	sKruhobot(sInt_32 id, const Properties &properties, const State &state);

	const State& get_State(void) const;
	void set_State(const State &state);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;
	virtual sResult from_Stream_mpfR(FILE *fr);	
	
    public:
	sInt_32 m_id;
	
	Properties m_properties;
	State m_state;
    };


/*----------------------------------------------------------------------------*/
// sRealConjunction

    class sRealConjunction
    {
    public:
	static const sInt_32 VACANT_LOCATION = 0; /* Kruhobots are numbered starting with 1 */
	static const sInt_32 UNDEFINED_LOCATION = -1;

	static const sInt_32 RANDOM_WALK_LENGTH = s__DEFAULT_RANDOM_WALK_LENGTH;

    public:
	typedef std::vector<sInt_32> LocationIDs_vector;

    public:
	sRealConjunction(s2DMap *Map);
	sRealConjunction(s2DMap *Map, sInt_32 N_kruhobots);
	/*----------------------------------------------------------------------------*/

	sInt_32 get_KruhobotCount(void) const;
	
	void place_Kruhobot(sInt_32 kruhobot_id, sInt_32 location_id);
	void remove_Kruhobot(sInt_32 kruhobot_id);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;
	virtual sResult from_Stream_mpfR(FILE *fr);

	virtual sResult from_File_xml_init(const sString &filename, sInt_32 N_kruhobots = -1);
	virtual sResult from_Stream_xml_init(FILE *fr, sInt_32 N_kruhobots = -1);

	virtual sResult from_File_xml_goal(const sString &filename, sInt_32 N_kruhobots = -1);
	virtual sResult from_Stream_xml_goal(FILE *fr, sInt_32 N_kruhobots = -1);

    public:
	s2DMap *m_Map;
	LocationIDs_vector m_kruhobot_Locations;
    };


/*----------------------------------------------------------------------------*/
// sRealInstance

    class sRealInstance
    {
    public:
	typedef std::vector<sKruhobot> Kruhobots_vector;
	typedef std::vector<sInt_32> LocationIDs_vector;	

    public:
	sRealInstance(s2DMap *Map);
	sRealInstance(const sRealConjunction &start_conjunction, const sRealConjunction &goal_conjunction);

    public:
	void add_Kruhobot(const sKruhobot &kruhobot);
	void add_Kruhobot(sInt_32 id, const sKruhobot &kruhobot);
        /*----------------------------------------------------------------------------*/

	void collect_StartingLocations(LocationIDs_vector &start_location_IDs) const;
	void collect_GoalLocations(LocationIDs_vector &goal_location_IDs) const;
	void collect_StartingGoalLocations(LocationIDs_vector &start_goal_location_IDs) const;	
        /*----------------------------------------------------------------------------*/

	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_mpfR(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_mpfR(const sString &filename);
	virtual sResult from_Stream_mpfR(FILE *fr);
        /*----------------------------------------------------------------------------*/

	virtual sResult from_File_movi(const sString &filename);
	virtual sResult from_Stream_movi(FILE *fr);	

	virtual sResult to_File_xml(const sString &filename, sInt_32 N_kruhobots = -1, const sString &indent = "") const;
	virtual void to_Stream_xml(FILE *fw, sInt_32 N_kruhobots = -1, const sString &indent = "") const;	
        /*----------------------------------------------------------------------------*/	
	
    public:
	Kruhobots_vector m_Kruhobots;
	    
	sRealConjunction m_start_conjunction;
	sRealConjunction m_goal_conjunction;	
    };


/*----------------------------------------------------------------------------*/
// sRealSolution

    class sRealSolution
    {
    public:
	static const sInt_32 N_MOTION_UNDEFINED = -1;

    public:
	struct Duration
	{
	    Duration() { /* nothing */ }
	    Duration(sDouble start_time, sDouble finish_time)
	    : m_start_time(start_time)
	    , m_finish_time(finish_time)
	    { /* nothing */ }
	    
	    sDouble m_start_time, m_finish_time;
	};
	
	struct Motion
	{	    
	    Motion()
	    { /* nothing */ }
	    
	    Motion(sInt_32 kruhobot_id,
	           sInt_32 src_loc_id,
		   sInt_32 dest_loc_id,
		   sDouble start_time,
		   sDouble finish_time)
	    : m_kruhobot_id(kruhobot_id)
	    , m_src_loc_id(src_loc_id)
	    , m_dest_loc_id(dest_loc_id)
	    , m_duration(start_time, finish_time)
	    { /* nothing */ }

	    Motion(sInt_32 kruhobot_id,
	           sInt_32 src_loc_id,
		   sInt_32 dest_loc_id,
		   const Duration &duration)
	    : m_kruhobot_id(kruhobot_id)
	    , m_src_loc_id(src_loc_id)
	    , m_dest_loc_id(dest_loc_id)
	    , m_duration(duration)
	    { /* nothing */ }	    

	    bool is_Undefined(void) const
	    {
		return (m_kruhobot_id < 0);
	    }
	    
	    bool operator<(const Motion &motion) const
	    {
		return (m_duration.m_start_time < motion.m_duration.m_start_time);
	    }
	    
	    bool operator==(const Motion &motion) const
	    {
		return (m_duration.m_start_time == motion.m_duration.m_start_time);
	    }

	    sInt_32 m_kruhobot_id;	    
	    sInt_32 m_src_loc_id;
	    sInt_32 m_dest_loc_id;
	    Duration m_duration;
	};

	static const Motion UNDEFINED_MOTION;	

	typedef std::set<Motion> Motions_set;
	
    public:
	sRealSolution();
	
	bool is_Null(void) const;
	sInt_32 get_MotionCount(void) const;

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	
        /*
	virtual void to_Screen_mpfR(const sString &indent = "") const;		
	virtual void to_Stream_mpfR(FILE *fw, const sString &indent = "") const;	

	virtual sResult to_File(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_mpfR(const sString &filename, const sString &indent = "") const;
	
	virtual sResult from_File_mpfR(const sString &filename);
	virtual sResult from_Stream_mpfR(FILE *fr);	
	*/
	
    public:
	Motions_set m_Motions;
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __KRUHO_R_H__ */
