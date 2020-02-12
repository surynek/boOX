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
/* cbsR_test.cpp / 1-224_leibniz                                              */
/*----------------------------------------------------------------------------*/
//
// Test of semi-continuous version of conflict-based search.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

#include "core/graph.h"
#include "core/agent.h"
#include "core/mapR.h"
#include "core/cbsR.h"

#include "util/statistics.h"

#include "test/cbsR_test.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    static const sDouble s_wait_factor = 0.2;

    
/*----------------------------------------------------------------------------*/

    void print_Introduction(void)
    {
	printf("----------------------------------------------------------------\n");
	printf("%s : Test of Semi-continuous Conflict-based Search\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");
    }

    
    void test_CBS_R_1(void)
    {
	printf("CBS-R test 1 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.populate_Network();
	square_map.to_Screen();

	sKruhobot kruhobot(1, sKruhobot::Properties(1.0, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot.to_Screen();

	sRealConjunction start_conjunction(&square_map, 1);
	start_conjunction.place_Kruhobot(1, 0);

	sRealConjunction goal_conjunction(&square_map, 1);
	goal_conjunction.place_Kruhobot(1, 3);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);
	instance.to_Screen();

	instance.add_Kruhobot(1, kruhobot);
	instance.to_Screen();
	
	sRealCBS::LocationConflicts__umap location_Conflicts;
	sRealCBS::LinearConflicts__map linear_Conflicts;
	sRealCBS::Schedule_vector kruhobot_Schedule;

	sRealCBS real_CBS(&instance);

	sRealCBS::LocationConflict location_conflict_0_1(0, sRealCBS::Interval(2.1, 2.2));
	sRealCBS::LocationConflict location_conflict_0_2(0, sRealCBS::Interval(3.3, 3.4));
	location_Conflicts[0].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_0_1.m_interval, location_conflict_0_1));
	location_Conflicts[0].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_0_2.m_interval, location_conflict_0_2));

	sRealCBS::LocationConflict location_conflict_1_1(1, sRealCBS::Interval(1.3, 1.4));
	sRealCBS::LocationConflict location_conflict_1_2(1, sRealCBS::Interval(0.5, 0.7));
	location_Conflicts[1].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_1_1.m_interval, location_conflict_1_1));
	location_Conflicts[1].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_1_2.m_interval, location_conflict_1_2));

	sRealCBS::LocationConflict location_conflict_2_1(2, sRealCBS::Interval(3.1, 3.4));
//	sRealCBS::LocationConflict location_conflict_2_2(2, sRealCBS::Interval(3.8, 4.7));
	location_Conflicts[2].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_2_1.m_interval, location_conflict_2_1));
//	location_Conflicts[2][location_conflict_2_2.m_interval] = location_conflict_2_2;

	sRealCBS::LocationConflict location_conflict_3_1(3, sRealCBS::Interval(0.0, 1.5));
	sRealCBS::LocationConflict location_conflict_3_2(3, sRealCBS::Interval(1.7, 3.6));
	sRealCBS::LocationConflict location_conflict_3_3(3, sRealCBS::Interval(5.7, 5.9));
	location_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_3_1.m_interval, location_conflict_3_1));
	location_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_3_2.m_interval, location_conflict_3_2));
	location_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_conflict_3_3.m_interval, location_conflict_3_3));

/*	
	sRealCBS::LocationConflict location_conflict_0_1(0, sRealCBS::Interval(1.0, 1.5));
	location_Conflicts[0][location_conflict_0_1.m_interval] = location_conflict_0_1;

	sRealCBS::LocationConflict location_conflict_1_1(0, sRealCBS::Interval(4.6, 4.7));
	location_Conflicts[1][location_conflict_1_1.m_interval] = location_conflict_1_1;

	sRealCBS::LocationConflict location_conflict_2_1(0, sRealCBS::Interval(4.6, 4.7));
	location_Conflicts[2][location_conflict_2_1.m_interval] = location_conflict_2_1;

	sRealCBS::LocationConflict location_conflict_3_1(0, sRealCBS::Interval(3.5, 5.7));
	location_Conflicts[3][location_conflict_3_1.m_interval] = location_conflict_3_1;
*/	
	real_CBS.find_KruhobotNonconflictingSchedule(kruhobot,
						     square_map,
						     0,
						     3,
						     10.0,
						     5.0,
						     location_Conflicts,
						     linear_Conflicts,
						     -1.0,
						     kruhobot_Schedule);
	
	printf("CBS-R test 1 ... finished\n");
    }


    void test_CBS_R_2(void)
    {
	printf("CBS-R test 2 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.populate_Network();	
	square_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(1.0, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(2.0, 1.5, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();
	

	sRealConjunction start_conjunction(&square_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);	

	sRealConjunction goal_conjunction(&square_map, 2);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 2);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);	
	instance.to_Screen();
	
	sRealCBS::LocationConflicts__umap location_1_Conflicts;
	sRealCBS::LinearConflicts__map linear_1_Conflicts;
	sRealCBS::Schedule_vector kruhobot_1_Schedule;

	sRealCBS real_CBS(&instance);

	sRealCBS::LocationConflict location_1_conflict_0_1(0, sRealCBS::Interval(2.1, 2.2));
	sRealCBS::LocationConflict location_1_conflict_0_2(0, sRealCBS::Interval(3.3, 3.4));
	location_1_Conflicts[0].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_0_1.m_interval, location_1_conflict_0_1));
	location_1_Conflicts[0].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_0_2.m_interval, location_1_conflict_0_2));

	sRealCBS::LocationConflict location_1_conflict_1_1(1, sRealCBS::Interval(0.3, 0.4));
	sRealCBS::LocationConflict location_1_conflict_1_2(1, sRealCBS::Interval(0.5, 0.7));
	location_1_Conflicts[1].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_1_1.m_interval, location_1_conflict_1_1));
	location_1_Conflicts[1].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_1_2.m_interval, location_1_conflict_1_2));

	sRealCBS::LocationConflict location_1_conflict_2_1(2, sRealCBS::Interval(3.1, 3.4));
//	sRealCBS::LocationConflict location_1_conflict_2_2(2, sRealCBS::Interval(3.8, 4.7));
	location_1_Conflicts[2].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_2_1.m_interval, location_1_conflict_2_1));
//	location_1_Conflicts[2][location_1_conflict_2_2.m_interval] = location_1_conflict_2_2;

	sRealCBS::LocationConflict location_1_conflict_3_1(3, sRealCBS::Interval(0.0, 1.5));
	sRealCBS::LocationConflict location_1_conflict_3_2(3, sRealCBS::Interval(1.7, 3.6));
	sRealCBS::LocationConflict location_1_conflict_3_3(3, sRealCBS::Interval(5.7, 5.9));
	location_1_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_3_1.m_interval, location_1_conflict_3_1));
	location_1_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_3_2.m_interval, location_1_conflict_3_2));
	location_1_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_1_conflict_3_3.m_interval, location_1_conflict_3_3));
	
	sRealCBS::LocationConflicts__umap location_2_Conflicts;
	sRealCBS::LinearConflicts__map linear_2_Conflicts;
	sRealCBS::Schedule_vector kruhobot_2_Schedule;

	sRealCBS::LocationConflict location_2_conflict_0_1(0, sRealCBS::Interval(2.1, 2.2));
	sRealCBS::LocationConflict location_2_conflict_0_2(0, sRealCBS::Interval(3.3, 3.4));
	location_2_Conflicts[0].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_0_1.m_interval, location_2_conflict_0_1));
	location_2_Conflicts[0].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_0_2.m_interval, location_2_conflict_0_2));

	sRealCBS::LocationConflict location_2_conflict_1_1(1, sRealCBS::Interval(1.3, 1.4));
	sRealCBS::LocationConflict location_2_conflict_1_2(1, sRealCBS::Interval(1.5, 1.7));
	location_2_Conflicts[1].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_1_1.m_interval, location_2_conflict_1_1));
	location_2_Conflicts[1].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_1_2.m_interval, location_2_conflict_1_2));

	sRealCBS::LocationConflict location_2_conflict_2_1(2, sRealCBS::Interval(3.1, 3.4));
//	sRealCBS::LocationConflict location_2_conflict_2_2(2, sRealCBS::Interval(3.8, 4.7));
	location_2_Conflicts[2].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_2_1.m_interval, location_2_conflict_2_1));
//	location_2_Conflicts[2][location_2_conflict_2_2.m_interval] = location_2_conflict_2_2;

	sRealCBS::LocationConflict location_2_conflict_3_1(3, sRealCBS::Interval(0.0, 1.5));
	sRealCBS::LocationConflict location_2_conflict_3_2(3, sRealCBS::Interval(1.7, 3.6));
	sRealCBS::LocationConflict location_2_conflict_3_3(3, sRealCBS::Interval(5.7, 5.9));
	location_2_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_3_1.m_interval, location_2_conflict_3_1));
	location_2_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_3_2.m_interval, location_2_conflict_3_2));
	location_2_Conflicts[3].insert(sRealCBS::LocationConflicts_map::value_type(location_2_conflict_3_3.m_interval, location_2_conflict_3_3));

	sDouble time_1 = real_CBS.find_KruhobotNonconflictingSchedule(kruhobot_1,
								      square_map,
								      0,
								      3,
								      10.0,
								      5.0,
								      location_1_Conflicts,
								      linear_1_Conflicts,
								      -1.0,
								      kruhobot_1_Schedule);

	sDouble time_2 = real_CBS.find_KruhobotNonconflictingSchedule(kruhobot_2,
								      square_map,
								      1,
								      2,
								      10.0,
								      5.0,
								      location_2_Conflicts,
								      linear_2_Conflicts,
								      -1.0,
								      kruhobot_2_Schedule);
	printf("Times: %.3f, %.3f\n", time_1, time_2);

	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;
	sRealCBS::KruhobotCollisions_mset kruhobot_Collisions;

	kruhobot_Schedules.push_back(sRealCBS::Schedule_vector());
	kruhobot_Schedules.push_back(kruhobot_1_Schedule);
	kruhobot_Schedules.push_back(kruhobot_2_Schedule);	

	real_CBS.analyze_NonconflictingSchedules(instance, kruhobot_Schedules, kruhobot_Collisions);
	printf("CBS-R test 2 ... finished\n");
    }


    void test_CBS_R_3(void)
    {
	printf("CBS-R test 3 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.populate_Network();	
	square_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();
	

	sRealConjunction start_conjunction(&square_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&square_map, 2);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 2);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      10.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 3 ... finished\n");
    }


    void test_CBS_R_4(void)
    {
	printf("CBS-R test 4 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.populate_Network();	
	square_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();		

	sRealConjunction start_conjunction(&square_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 3);	

	sRealConjunction goal_conjunction(&square_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 3);
	goal_conjunction.place_Kruhobot(3, 2);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);		
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      10.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 4 ... finished\n");
    }


    void test_CBS_R_5(void)
    {
	printf("CBS-R test 5 ...\n");	

	s2DMap crossing_map(7);

	crossing_map.add_Location(0, -2.0, 0.0);
	crossing_map.add_Location(1, -1.0, 0.0);
	crossing_map.add_Location(2, 0.0, 0.0);
    	crossing_map.add_Location(3, 1.0, 0.0);
    	crossing_map.add_Location(4, 2.0, 0.0);
    
    	crossing_map.add_Location(5, 0.0, 1.0);
    	crossing_map.add_Location(6, 0.0, 2.0);    
    
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.6, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.6, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&crossing_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 6);

	sRealConjunction goal_conjunction(&crossing_map, 2);
	goal_conjunction.place_Kruhobot(1, 4);
	goal_conjunction.place_Kruhobot(2, 5);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      10.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 5 ... finished\n");
    }


    void test_CBS_R_6(void)
    {
	printf("CBS-R test 6 ...\n");	

	s2DMap crossing_map(7);

	crossing_map.add_Location(0, -2.0, 0.0);
	crossing_map.add_Location(1, -1.0, 0.0);
	crossing_map.add_Location(2, 0.0, 0.0);
    	crossing_map.add_Location(3, 1.0, 0.0);
    	crossing_map.add_Location(4, 2.0, 0.0);
    
    	crossing_map.add_Location(5, 0.0, 1.0);
    	crossing_map.add_Location(6, 0.0, 2.0);    
    
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&crossing_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&crossing_map, 2);
	goal_conjunction.place_Kruhobot(1, 4);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 6 ... finished\n");
    }


    void test_CBS_R_7(void)
    {
	printf("CBS-R test 7 ...\n");	

	s2DMap crossing_map(9);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

    	crossing_map.add_Location(3, 0.0, 1.0);	
    	crossing_map.add_Location(4, 1.0, 1.0);	
	crossing_map.add_Location(5, 2.0, 1.0);
    
    	crossing_map.add_Location(6, 0.0, 2.0);	
    	crossing_map.add_Location(7, 1.0, 2.0);	
    	crossing_map.add_Location(8, 2.0, 2.0);
/*
    	crossing_map.add_Location(8, 0.0, 3.0);	
    	crossing_map.add_Location(9, 1.0, 3.0);	
    	crossing_map.add_Location(10, 2.0, 3.0);	
*/	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);
	start_conjunction.place_Kruhobot(3, 3);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 8);
	goal_conjunction.place_Kruhobot(2, 7);
	goal_conjunction.place_Kruhobot(3, 6);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 7 ... finished\n");
    }


    void test_CBS_R_8(void)
    {
	printf("CBS-R test 8 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.populate_Network();	
	square_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&square_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&square_map, 2);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 1);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 8 ... finished\n");
    }    


    void test_CBS_R_9(void)
    {
	printf("CBS-R test 9 ...\n");	

	s2DMap crossing_map(6);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

    	crossing_map.add_Location(3, 0.0, 1.0);	
    	crossing_map.add_Location(4, 1.0, 1.0);	
    	crossing_map.add_Location(5, 2.0, 1.0);
    	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.01, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.01, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.01, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 5);
	goal_conjunction.place_Kruhobot(2, 4);
	goal_conjunction.place_Kruhobot(3, 3);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test 9 ... finished\n");
    }
 

    void test_CBS_R_A(void)
    {
	printf("CBS-R test A ...\n");	

	s2DMap crossing_map(6);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

//    	crossing_map.add_Location(3, 20.0, 1.0);	
    
    	crossing_map.add_Location(3, 0.0, 2.0);	
    	crossing_map.add_Location(4, 1.0, 2.0);	
    	crossing_map.add_Location(5, 2.0, 2.0);
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 5);
	goal_conjunction.place_Kruhobot(2, 4);
	goal_conjunction.place_Kruhobot(3, 3);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test A ... finished\n");
    }


    void test_CBS_R_B(void)
    {
	printf("CBS-R test B ...\n");	

	s2DMap crossing_map(6);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);

	crossing_map.add_Location(2, 0.0, 1.0);
	crossing_map.add_Location(3, 1.0, 1.0);

	crossing_map.add_Location(4, 0.0, 2.0);
	crossing_map.add_Location(5, 1.0, 2.0);
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&crossing_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&crossing_map, 2);
	goal_conjunction.place_Kruhobot(1, 5);
	goal_conjunction.place_Kruhobot(2, 4);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test B ... finished\n");
    }


    void test_CBS_R_C(void)
    {
	printf("CBS-R test C ...\n");	

	s2DMap crossing_map(7);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

    	crossing_map.add_Location(3, 1.0, 1.0);	
    
    	crossing_map.add_Location(4, 0.0, 2.0);	
    	crossing_map.add_Location(5, 1.0, 2.0);	
    	crossing_map.add_Location(6, 2.0, 2.0);
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 6);
	goal_conjunction.place_Kruhobot(2, 5);
	goal_conjunction.place_Kruhobot(3, 4);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test C ... finished\n");
    }


    void test_CBS_R_D(void)
    {
	printf("CBS-R test D ...\n");	

	s2DMap crossing_map(8);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

    	crossing_map.add_Location(3, 0.0, 1.0);
    	crossing_map.add_Location(4, 2.0, 1.0);		
    
    	crossing_map.add_Location(5, 0.0, 2.0);	
    	crossing_map.add_Location(6, 1.0, 2.0);	
    	crossing_map.add_Location(7, 2.0, 2.0);
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

 	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 7);
	goal_conjunction.place_Kruhobot(2, 6);
	goal_conjunction.place_Kruhobot(3, 5);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test D ... finished\n");
    }        


    void test_CBS_R_E(void)
    {
	printf("CBS-R test E ...\n");	

	s2DMap crossing_map(8);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);
    	crossing_map.add_Location(3, 3.0, 0.0);
	
    	crossing_map.add_Location(4, 0.0, 1.0);		    
    	crossing_map.add_Location(5, 1.0, 1.0);	
    	crossing_map.add_Location(6, 2.0, 1.0);	
    	crossing_map.add_Location(7, 3.0, 1.0);
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network(3.0);	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();

	sKruhobot kruhobot_4(4, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_4.to_Screen();

	sRealConjunction start_conjunction(&crossing_map, 4);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);
	start_conjunction.place_Kruhobot(4, 3);		

	sRealConjunction goal_conjunction(&crossing_map, 4);
	goal_conjunction.place_Kruhobot(1, 5);
	goal_conjunction.place_Kruhobot(2, 6);
	goal_conjunction.place_Kruhobot(3, 7);
	goal_conjunction.place_Kruhobot(4, 4);		

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.add_Kruhobot(4, kruhobot_4);	
	instance.to_Screen();

	instance.to_File_mpfR("layered_E.kruR");
	crossing_map.to_File_mapR("layered_E.mapR");

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test E ... finished\n");
    }


    void test_CBS_R_F(void)
    {
	printf("CBS-R test F ...\n");	

	s2DMap crossing_map(5);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

	crossing_map.add_Location(3, 0.5, 1.0);
	crossing_map.add_Location(4, 1.5, 1.0);		
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 4);
	goal_conjunction.place_Kruhobot(3, 0);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      100.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test F ... finished\n");
    }


    void test_CBS_R_G(void)
    {
	printf("CBS-R test G ...\n");	

	s2DMap crossing_map(5);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

	crossing_map.add_Location(3, 0.5, 1.0);
	crossing_map.add_Location(4, 1.5, 1.0);		
	
	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network(1.9);	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();

	sKruhobot kruhobot_4(4, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_4.to_Screen();		

	sRealConjunction start_conjunction(&crossing_map, 4);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);
	start_conjunction.place_Kruhobot(4, 3);		

	sRealConjunction goal_conjunction(&crossing_map, 4);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 4);
	goal_conjunction.place_Kruhobot(3, 0);
	goal_conjunction.place_Kruhobot(4, 1);		

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);
	instance.add_Kruhobot(4, kruhobot_4);		
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);		
	printf("Found cost: %.3f\n", solution_cost);

	printf("CBS-R test G ... finished\n");
    }


    void test_CBS_R_H(void)
    {
	printf("CBS-R test H ...\n");	

	s2DMap crossing_map(4);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 0.0, 1.0);
	crossing_map.add_Location(3, 1.0, 1.0);	

	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();

	sKruhobot kruhobot_4(4, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_4.to_Screen();		

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 2);
	goal_conjunction.place_Kruhobot(3, 1);
	
	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      0.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);	

	printf("CBS-R test H ... finished\n");
    }            
    

    void test_CBS_R_I(void)
    {
	printf("CBS-R test I ...\n");	

	s2DMap crossing_map(5);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 0.0, 1.0);
	crossing_map.add_Location(3, 1.0, 1.0);
	crossing_map.add_Location(4, 3.3, 3.3);		

	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.6, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.6, 1.5, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();

	sKruhobot kruhobot_4(4, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_4.to_Screen();		

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 0);
	goal_conjunction.place_Kruhobot(2, 2);
	goal_conjunction.place_Kruhobot(3, 1);
	
	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      0.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);	

	printf("CBS-R test I ... finished\n");
    }


    void test_CBS_R_J(void)
    {
	printf("CBS-R test J ...\n");	

	s2DMap crossing_map(3);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);

	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&crossing_map, 2);
	start_conjunction.place_Kruhobot(1, 1);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&crossing_map, 2);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 0);
	
	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      0.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("CBS-R test J ... finished\n");
    }


    void test_CBS_R_K(void)
    {
	printf("CBS-R test K ...\n");	

	s2DMap crossing_map(4);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);
	crossing_map.add_Location(3, 1.0, 1.0);	

	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 2);
	goal_conjunction.place_Kruhobot(2, 1);
	goal_conjunction.place_Kruhobot(3, 0);	
	
	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      9.0,
					      0.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);	

	printf("CBS-R test K ... finished\n");
    }


    void test_CBS_R_L(void)
    {
	printf("CBS-R test L ...\n");	

	s2DMap crossing_map(4);

	crossing_map.add_Location(0, 0.0, 0.0);
	crossing_map.add_Location(1, 1.0, 0.0);
	crossing_map.add_Location(2, 2.0, 0.0);
	crossing_map.add_Location(3, 1.0, 1.0);	

	crossing_map.calc_AllPairsStraightDistances();
	crossing_map.populate_Network();	
	crossing_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(1.0, 2.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.2, 2.0, 2.0, M_PI / 3, M_PI / 7, s_wait_factor), sKruhobot::State(M_PI / 5, sKruhobot::Position(3.0, 4.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.3, 3.0, 3.0, M_PI / 2, M_PI / 8, s_wait_factor), sKruhobot::State(M_PI / 6, sKruhobot::Position(5.0, 6.0)));
	kruhobot_3.to_Screen();	

	sRealConjunction start_conjunction(&crossing_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);	

	sRealConjunction goal_conjunction(&crossing_map, 3);
	goal_conjunction.place_Kruhobot(1, 2);
	goal_conjunction.place_Kruhobot(2, 1);
	goal_conjunction.place_Kruhobot(3, 0);	
	
	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);	
	instance.to_Screen();

	instance.to_File_mpfR("small002.kruR");
	crossing_map.to_File_mapR("small002.mapR");

	s2DMap loaded_map;
	loaded_map.from_File_mapR("small002.mapR");
	loaded_map.calc_AllPairsStraightDistances();
	
	printf("Loaded map\n");
	loaded_map.to_Screen();

		
	sRealInstance loaded_instance(&loaded_map);	
	loaded_instance.from_File_mpfR("small002.kruR");

	printf("Loaded real instance\n");
	loaded_instance.to_Screen();	

	printf("CBS-R test L ... finished\n");
    }


    void test_CBS_R_E8(void)
    {
	printf("SMT-CBS-R test E8 ...\n");	

	s2DMap grid_map(16);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.0);
	grid_map.add_Location(2, 2.0, 0.0);
	grid_map.add_Location(3, 3.0, 0.0);	

	grid_map.add_Location(4, 0.0, 1.0);
	grid_map.add_Location(5, 1.0, 1.0);
	grid_map.add_Location(6, 2.0, 1.0);
	grid_map.add_Location(7, 3.0, 1.0);	

	grid_map.add_Location(8,  0.0, 2.0);
	grid_map.add_Location(9,  1.0, 2.0);
	grid_map.add_Location(10, 2.0, 2.0);
	grid_map.add_Location(11, 3.0, 2.0);

	grid_map.add_Location(12, 0.0, 3.0);
	grid_map.add_Location(13, 1.0, 3.0);
	grid_map.add_Location(14, 2.0, 3.0);
	grid_map.add_Location(15, 3.0, 3.0);	       		
	    
	grid_map.calc_AllPairsStraightDistances();
	grid_map.populate_Network(1.5);	
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();

	sKruhobot kruhobot_4(4, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_4.to_Screen();	

	sRealConjunction start_conjunction(&grid_map, 4);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);
	start_conjunction.place_Kruhobot(4, 3);	

	sRealConjunction goal_conjunction(&grid_map, 4);
	goal_conjunction.place_Kruhobot(1, 15);
	goal_conjunction.place_Kruhobot(2, 10);
	goal_conjunction.place_Kruhobot(3, 11);
	goal_conjunction.place_Kruhobot(4, 12);	

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);
	instance.add_Kruhobot(4, kruhobot_4);	
	instance.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_CBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      9.0,
					      0.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);

	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
	
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);	

	printf("CBS-R test 8E ... finished\n");
    }


    void test_Collision_1(void)
    {
	printf("CBS-R collision 1 ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.25);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
//	sRealCBS::Traversal wait_traversal(2, 1, 1, sRealCBS::Interval(1.2, 1.7));
	sRealCBS::Traversal wait_traversal(2, 1, 1, sRealCBS::Interval(0.999, 1.5));
	sRealCBS::Traversal pass_traversal(1, 0, 2, sRealCBS::Interval(0.0, 2.0));

	wait_traversal.to_Screen();
	pass_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision_location_X_linear(instance,
							     wait_traversal,
							     pass_traversal,
							     kruhobot_location_Conflicts,
							     kruhobot_linear_Conflicts,
							     last_conflict_id,
							     false);
	
	printf("CBS-R collision 1 ... finished\n");
    }


    void test_Collision_1B(void)
    {
	printf("CBS-R collision 1B ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.25);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
//	sRealCBS::Traversal wait_traversal(2, 1, 1, sRealCBS::Interval(1.2, 1.7));
	sRealCBS::Traversal wait_traversal(2, 1, 1, sRealCBS::Interval(1.999, 2.001));	
	sRealCBS::Traversal pass_traversal(1, 0, 2, sRealCBS::Interval(1.0, 3.0));

	wait_traversal.to_Screen();
	pass_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision_location_X_linear(instance,
							     wait_traversal,
							     pass_traversal,
							     kruhobot_location_Conflicts,
							     kruhobot_linear_Conflicts,
							     last_conflict_id,
							     false);
	
	printf("CBS-R collision 1B ... finished\n");
    }    


    void test_Collision_2(void)
    {
	printf("CBS-R collision 2 ...\n");	

	s2DMap grid_map(4);

	grid_map.add_Location(0, 0.0, 1.0);
	grid_map.add_Location(1, 2.0, 1.0);
	grid_map.add_Location(2, 1.0, 0.0);
	grid_map.add_Location(3, 1.0, 2.0);	

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 3);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.0, 2.0));
	sRealCBS::Traversal pass2_traversal(2, 2, 3, sRealCBS::Interval(0.0, 2.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision_linear_X_linear(instance,
							   pass1_traversal,
							   pass2_traversal,
							   kruhobot_location_Conflicts,
							   kruhobot_linear_Conflicts,
							   last_conflict_id,
							   false);
	
	printf("CBS-R collision 2 ... finished\n");
    }


    void test_Collision_3(void)
    {
	printf("CBS-R collision 3 ...\n");	

	s2DMap grid_map(4);

	grid_map.add_Location(0, 0.0, 1.0);
	grid_map.add_Location(1, 2.0, 1.0);
	grid_map.add_Location(2, 1.0, 0.0);
	grid_map.add_Location(3, 1.0, 2.0);	

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 3);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.707, 2.707));
	sRealCBS::Traversal pass2_traversal(2, 2, 3, sRealCBS::Interval(0.0, 2.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision_linear_X_linear(instance,
							   pass1_traversal,
							   pass2_traversal,
							   kruhobot_location_Conflicts,
							   kruhobot_linear_Conflicts,
							   last_conflict_id,
							   false);
	
	printf("CBS-R collision 3 ... finished\n");
    }


    void test_Collision_4(void)
    {
	printf("CBS-R collision 4 ...\n");	

	s2DMap grid_map(4);

	grid_map.add_Location(0, 0.0, 1.0);
	grid_map.add_Location(1, 2.0, 1.0);
	grid_map.add_Location(2, 1.0, 0.0);
	grid_map.add_Location(3, 1.0, 2.0);	

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.25, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 3);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.0, 2.0));
	sRealCBS::Traversal pass2_traversal(2, 2, 3, sRealCBS::Interval(0.707, 2.707));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision_linear_X_linear(instance,
							   pass1_traversal,
							   pass2_traversal,
							   kruhobot_location_Conflicts,
							   kruhobot_linear_Conflicts,
							   last_conflict_id,
							   false);
	
	printf("CBS-R collision 4 ... finished\n");
    }


    void test_Collision_5(void)
    {
	printf("CBS-R collision 5 ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.0);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.2, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.2, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 2);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 2);
	goal_conjunction.place_Kruhobot(2, 0);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.0, 1.0));
	sRealCBS::Traversal pass2_traversal(2, 2, 1, sRealCBS::Interval(0.0, 1.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	real_CBS.resolve_KruhobotCollision_linear_X_linear(instance,
							   pass1_traversal,
							   pass2_traversal,
							   kruhobot_location_Conflicts,
							   kruhobot_linear_Conflicts,
							   last_conflict_id,
							   false);
	
	printf("CBS-R collision 5 ... finished\n");
    }                
    

    void test_Collision_6(void)
    {
	printf("CBS-R collision 6 ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.0);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.2, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.2, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.0, 1.0));
	sRealCBS::Traversal pass2_traversal(2, 1, 2, sRealCBS::Interval(0.0, 1.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	real_CBS.resolve_KruhobotCollision_linear_X_linear(instance,
							   pass1_traversal,
							   pass2_traversal,
							   kruhobot_location_Conflicts,
							   kruhobot_linear_Conflicts,
							   last_conflict_id,
							   false);
	
	printf("CBS-R collision 6 ... finished\n");
    }                


    void test_Collision_7(void)
    {
	printf("CBS-R collision 7 ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.0);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.2, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.2, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 1, 0, sRealCBS::Interval(0.0, 1.0));
	sRealCBS::Traversal pass2_traversal(2, 1, 2, sRealCBS::Interval(0.0, 1.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	real_CBS.resolve_KruhobotCollision_linear_X_linear(instance,
							   pass1_traversal,
							   pass2_traversal,
							   kruhobot_location_Conflicts,
							   kruhobot_linear_Conflicts,
							   last_conflict_id,
							   false);
	
	printf("CBS-R collision 7 ... finished\n");
    }                


    void test_Collision_8(void)
    {
	printf("CBS-R collision 8 ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 0.0, 1.0);
	grid_map.add_Location(2, 1.0, 1.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.0, 1.0));
	sRealCBS::Traversal pass2_traversal(2, 1, 2, sRealCBS::Interval(0.0, 1.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	printf("CBS-R collision 8 ... finished\n");
    }


    void test_Collision_9(void)
    {
	printf("CBS-R collision 9...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 0.0, 1.0);
	grid_map.add_Location(2, 1.0, 1.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.132, 1.132));
	sRealCBS::Traversal pass2_traversal(2, 1, 2, sRealCBS::Interval(0.0, 1.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	printf("CBS-R collision 9... finished\n");
    }


    void test_Collision_10(void)
    {
	printf("CBS-R collision 10...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 0.0, 1.0);
	grid_map.add_Location(2, 1.0, 1.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.1, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.9, 2.7));
	sRealCBS::Traversal pass2_traversal(2, 1, 2, sRealCBS::Interval(0.0, 1.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	printf("CBS-R collision 10... finished\n");
    }


    void test_Collision_11(void)
    {
	printf("CBS-R collision 10...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.0);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.112, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;
	
	sRealCBS::Traversal pass1_traversal(2, 0, 1, sRealCBS::Interval(0.899 + 0.00, 1.799 + 0.00));
	sRealCBS::Traversal pass2_traversal(1, 1, 2, sRealCBS::Interval(1.0, 2.0));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	printf("CBS-R collision 11... finished\n");
    }            



    void test_Collision_12(void)
    {
	printf("CBS-R collision 12...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 1.0);
	grid_map.add_Location(1, 1.0, 1.0);
	grid_map.add_Location(2, 2.0, 1.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.112, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;

	sDouble t0 = 0;
	sDouble t1 = 0;	
	
	sRealCBS::Traversal pass1_traversal(1, 1, 2, sRealCBS::Interval(1.0 + t0, 2.0 + t0));
	sRealCBS::Traversal pass2_traversal(2, 1, 1, sRealCBS::Interval(1.799 + t1, 2.0 + t1));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	printf("CBS-R collision 12... finished\n");
    }                


    void test_Collision_13(void)
    {
	printf("CBS-R collision 13 ...\n");	

	s2DMap grid_map(3);

	grid_map.add_Location(0, 0.0, 1.0);
	grid_map.add_Location(1, 1.0, 1.0);
	grid_map.add_Location(2, 2.0, 1.0);

	grid_map.populate_Network(1.1);
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.4, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 1);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	const sInt_32 N_kruhobots = 2;

	sDouble t0 = 0;
	sDouble t1 = 0;	
	
	sRealCBS::Traversal pass1_traversal(1, 0, 1, sRealCBS::Interval(0.0 + t0, 1.0 + t0));
	sRealCBS::Traversal pass2_traversal(2, 2, 1, sRealCBS::Interval(0.0 + t1, 1.0 + t1));

	pass1_traversal.to_Screen();
	pass2_traversal.to_Screen();

	sRealCBS real_CBS(&instance);
	sRealCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	
	sRealCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;
	kruhobot_linear_Conflicts.resize(N_kruhobots + 1);

	sInt_32 last_conflict_id = 0;

	real_CBS.resolve_KruhobotCollision(instance,
					   pass1_traversal,
					   pass2_traversal,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts,
					   last_conflict_id,
					   false);

	printf("CBS-R collision 13 ... finished\n");
    }


    #define MIN(x,y) (((x) < (y)) ? (x) : (y))
    #define MAX(x,y) (((x) > (y)) ? (x) : (y))    
    
    struct Interval
    {
	Interval() { /* nothing */ }
	Interval(double lower, double upper)
	    : m_lower(lower)
	    , m_upper(upper)
	{ /* nothing */ }
	
	    double size(void) const
	    {		
		return (m_upper - m_lower);
	    }	    	
	    Interval intersect(const Interval &interval) const
	    {
		return Interval(MAX(m_lower, interval.m_lower), MIN(m_upper, interval.m_upper));
	    }
	
	    double m_lower, m_upper;
    };
    

    const double EPSILON = 0.000000001;
    const double DELTA = 0.0001;

    std::pair<bool, bool> calculate_UnsafeIntervals(
	double               A_x1,  /* origin of agent A */
	double               A_y1,
	double               A_x2,  /* destination of agent A */
	double               A_y2,
	const Interval      &A_time, /* time interval of agent A */
	double               B_x1,  /* origin of agent B */
	double               B_y1,
	double               B_x2,  /* destination of agent B */
	double               B_y2,
	const Interval      &B_time,  /* time interval of agent B */
	double               A_v,   /* velocity of agent A */
	double               A_r,   /* radius of agent A */
	double               B_v,   /* velocity of agent B */
	double               B_r,   /* radius of agent B */				   
	Interval            &unsafe_A, /* unsafe time interval for A */
	Interval            &unsafe_B) /* unsafe time interval for B */
    /* returns true in some element of the pair if some avoidance needed, false/false if it is safe to go ahead for both agents */
    /* time intervals, origins and destinations must be consistent together with velocities */
    {
	bool avoid_A = false;
	bool avoid_B = false;
	
	double vA = A_v;
	double rA = A_r;
	
	double vB = B_v;
	double rB = B_r;

	double x1A_ = A_x1;
	double x2A_ = A_x2;
	double y1A_ = A_y1;
	double y2A_ = A_y2;
	
	double x1B_ = B_x1;
	double x2B_ = B_x2;
	double y1B_ = B_y1;
	double y2B_ = B_y2;

	double dxA = x2A_ - x1A_;
	double dyA = y2A_ - y1A_;

	double dxB = x2B_ - x1B_;
	double dyB = y2B_ - y1B_;

	double d2A = dxA * dxA + dyA * dyA;
	double dA = sqrt(d2A);

	double d2B = dxB * dxB + dyB * dyB;
	double dB = sqrt(d2B);

	double x1A = x1A_ + (vA * (-A_time.m_lower) * dxA) / dA;
	double y1A = y1A_ + (vA * (-A_time.m_lower) * dyA) / dA;

	double x1B = x1B_ + (vB * (-B_time.m_lower) * dxB) / dB;
	double y1B = y1B_ + (vB * (-B_time.m_lower) * dyB) / dB;

	double dxAB = x1A - x1B;
	double dyAB = y1A - y1B;
	double rAB = rA + rB;

	{
	    double gamma = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
	    double beta_x = vA * dxA / dA - vB * dxB / dB;
	    double beta_y = vA * dyA / dA - vB * dyB / dB;
	    double beta = 2 * (dxAB * beta_x + dyAB * beta_y);
	    double alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    double discriminant = beta * beta - 4 * alpha * gamma;
		
	    if (discriminant <= EPSILON)
	    {
		return std::pair<bool, bool>(false, false);
	    }
	    else
	    {
		double tau_1 = (-beta - sqrt(discriminant)) / (2 * alpha); 
		double tau_2 = (-beta + sqrt(discriminant)) / (2 * alpha);

		Interval check_intersection_A = A_time.intersect(Interval(tau_1, tau_2));
		Interval check_intersection_B = B_time.intersect(Interval(tau_1, tau_2));

		if (check_intersection_A.size() <= DELTA || check_intersection_B.size() <= DELTA)
		{
		    return std::pair<bool, bool>(false, false);
		}
	    }
	}
	    
	{
	    double gamma_0 = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
	    double gamma_1 = 2 * (vA * (dxAB * dxA + dyAB * dyA)) / dA;	
	    double gamma_2 = vA * vA;

	    double beta_0 = 2 * ((vA * dxAB * dxA / dA) - ((vB * dxAB * dxB) / dB) + (vA * dyAB * dyA / dA) - ((vB * dyAB * dyB) / dB));
	    double beta_x = (vA * dxA / dA) - (vB * dxB / dB);
	    double beta_y = (vA * dyA / dA) - (vB * dyB / dB);	
	    double beta_1 = 2 * ((vA * dxA * beta_x) / dA + (vA * dyA * beta_y) / dA);

	    double alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    double lower_unsafe_A, upper_unsafe_A;
	
	    double A = beta_1 * beta_1 - 4 * alpha * gamma_2;
	    double B = 2 * beta_0 * beta_1 - 4 * alpha * gamma_1;
	    double C = beta_0 * beta_0 - 4 * alpha * gamma_0;
	    
	    if (sABS(A) > EPSILON)
	    {
		double D = B * B - 4 * A * C;

		if (D > EPSILON)
		{
		    double t0_1 = (-B - sqrt(D)) / (2 * A); 
		    double t0_2 = (-B + sqrt(D)) / (2 * A);

		    lower_unsafe_A = -t0_1 + A_time.m_lower;		    
		    upper_unsafe_A = -t0_2 + A_time.m_lower;
		    
		    Interval avoid_interval_A(sMAX(0, sMIN(lower_unsafe_A, upper_unsafe_A)), sMAX(0, sMAX(lower_unsafe_A, upper_unsafe_A)));

		    if (avoid_interval_A.size() > DELTA)
		    {
			Interval intersection_A = B_time.intersect(avoid_interval_A);
			
			if (intersection_A.size() > DELTA)
			{
			    unsafe_A = intersection_A;
			    avoid_A = true;
			}
		    }
		}
	    }
	    else
	    {
		double gamma = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
		double beta_x = vA * dxA / dA - vB * dxB / dB;
		double beta_y = vA * dyA / dA - vB * dyB / dB;
		double beta = 2 * (dxAB * beta_x + dyAB * beta_y);
		double alpha = beta_x * beta_x + beta_y * beta_y;
		
		double disc = beta * beta - 4 * alpha * gamma;

		if (disc > EPSILON)
		{
		    double time0_1 = (-beta - sqrt(disc)) / (2 * alpha); 
		    double time0_2 = (-beta + sqrt(disc)) / (2 * alpha);

		    lower_unsafe_A = time0_1;
		    upper_unsafe_A = time0_2;

		    Interval avoid_interval_A(sMAX(0, sMIN(lower_unsafe_A, upper_unsafe_A)), sMAX(0, sMAX(lower_unsafe_A, upper_unsafe_A)));

		    if (avoid_interval_A.size() > DELTA)
		    {
			Interval intersection_A = A_time.intersect(avoid_interval_A);
		    
			if (intersection_A.size() > DELTA)
			{
			    double AA = vA * vA + vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    double BB0 = 2 * (vA * (dxAB * dxA + dyAB * dyA) / dA - vB * (dxAB * dxB + dyAB * dyB) / dB);
			    double BB1 = 2 * vA * vA - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    double CC0 = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
			    double CC1 = 2 * vA * (dxAB * dxA + dyAB * dyA) / dA;
			    double CC2 = vA * vA;

			    double tB2 = B_time.m_upper;

			    double A = AA * CC2;
			    double B = AA * BB1 * tB2 + AA * CC1;
			    double C = AA * BB0 * tB2 + AA * CC0 + AA * AA * tB2 * tB2;

			    double DD = B * B - 4 * A * C;

			    if (DD > EPSILON)
			    {
				double T0_1 = (-B - sqrt(DD)) / (2 * A); 
				double T0_2 = (-B + sqrt(DD)) / (2 * A);
				
				double lower_unsafe_A_ = A_time.m_lower - MIN(0, T0_1);
				double upper_unsafe_A_ = A_time.m_lower - MIN(0, T0_2);

				Interval avoid_interval_A_(sMAX(0, sMIN(lower_unsafe_A_, upper_unsafe_A_)), sMAX(0, sMAX(lower_unsafe_A_, upper_unsafe_A_)));
				
				if (avoid_interval_A_.size() > DELTA)
				{
				    Interval intersection_A_ = A_time.intersect(avoid_interval_A_);
				    
				    if (intersection_A_.size() > DELTA)
				    {
					unsafe_A = intersection_A_;
					avoid_A = true;
				    }
				}
			    }
			    else
			    {
				unsafe_A = A_time;
				avoid_A = true;
			    }
			}
		    }		    
		}		
	    }
	}

	double dxBA = x1B - x1A;
	double dyBA = y1B - y1A;

	{
	    double gamma_0 = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
	    double gamma_1 = 2 * (vB * (dxBA * dxB + dyBA * dyB)) / dB;	
	    double gamma_2 = vB * vB;

	    double beta_0 = 2 * ((vB * dxBA * dxB / dB) - ((vA * dxBA * dxA) / dA) + (vB * dyBA * dyB / dB) - ((vA * dyBA * dyA) / dA));
	    double beta_x = (vB * dxB / dB) - (vA * dxA / dA);
	    double beta_y = (vB * dyB / dB) - (vA * dyA / dA);
	    double beta_1 = 2 * ((vB * dxB * beta_x) / dB + (vB * dyB * beta_y) / dB);	    

	    double alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    double lower_unsafe_B, upper_unsafe_B;
	
	    double A = beta_1 * beta_1 - 4 * alpha * gamma_2;
	    double B = 2 * beta_0 * beta_1 - 4 * alpha * gamma_1;
	    double C = beta_0 * beta_0 - 4 * alpha * gamma_0;

	    if (sABS(A) > EPSILON)
	    {
		double D = B * B - 4 * A * C;

		if (D > EPSILON)
		{
		    double t0_1 = (-B - sqrt(D)) / (2 * A); 
		    double t0_2 = (-B + sqrt(D)) / (2 * A);

		    lower_unsafe_B = -t0_1 + B_time.m_lower;
		    upper_unsafe_B = -t0_2 + B_time.m_lower;

		    Interval avoid_interval_B(sMAX(0, sMIN(lower_unsafe_B, upper_unsafe_B)), sMAX(0, sMAX(lower_unsafe_B, upper_unsafe_B)));

		    if (avoid_interval_B.size() > DELTA)
		    {
			Interval intersection_B = A_time.intersect(avoid_interval_B);
		    
			if (intersection_B.size() > DELTA)
			{
			    unsafe_B = intersection_B;
			    avoid_B = true;
			}
		    }
		}
	    }
	    else
	    {
		double gamma = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
		double beta_x = vB * dxB / dB - vA * dxA / dA;
		double beta_y = vB * dyB / dB - vA * dyA / dA;
		double beta = 2 * (dxBA * beta_x + dyBA * beta_y);
		double alpha = beta_x * beta_x + beta_y * beta_y;		

		double disc = beta * beta - 4 * alpha * gamma;

		if (disc > EPSILON)
		{
		    double time0_1 = (-beta - sqrt(disc)) / (2 * alpha); 
		    double time0_2 = (-beta + sqrt(disc)) / (2 * alpha);

		    lower_unsafe_B = time0_1;
		    upper_unsafe_B = time0_2;		    

		    Interval avoid_interval_B(sMAX(0, sMIN(lower_unsafe_B, upper_unsafe_B)), sMAX(0, sMAX(lower_unsafe_B, upper_unsafe_B)));

		    if (avoid_interval_B.size() > DELTA)
		    {
			Interval intersection_B = B_time.intersect(avoid_interval_B);
		    
			if (intersection_B.size() > DELTA)
			{
			    double AA = vA * vA + vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    double BB0 = 2 * (vB * (dxBA * dxB + dyBA * dyB) / dB - vA * (dxBA * dxA + dyBA * dyA) / dA);
			    double BB1 = 2 * vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);			    
			    double CC0 = dxBA * dxBA + dyBA * dyBA - rAB * rAB;			    
			    double CC1 = 2 * vB * (dxBA * dxB + dyBA * dyB) / dB;
			    double CC2 = vB * vB;

			    double tB2 = A_time.m_upper;

			    double A = AA * CC2;
			    double B = AA * BB1 * tB2 + AA * CC1;
			    double C = AA * BB0 * tB2 + AA * CC0 + AA * AA * tB2 * tB2;

			    double DD = B * B - 4 * A * C;

			    if (DD > EPSILON)
			    {
				double T0_1 = (-B - sqrt(DD)) / (2 * A); 
				double T0_2 = (-B + sqrt(DD)) / (2 * A);
			    
				double lower_unsafe_B_ = B_time.m_lower - sMIN(0, T0_1);
				double upper_unsafe_B_ = B_time.m_lower - sMIN(0, T0_2);
				
				Interval avoid_interval_B_(sMAX(0, sMIN(lower_unsafe_B_, upper_unsafe_B_)), sMAX(0, sMAX(lower_unsafe_B_, upper_unsafe_B_)));
				
				if (avoid_interval_B_.size() > DELTA)
				{
				    Interval intersection_B_ = A_time.intersect(avoid_interval_B_);
				    
				    if (intersection_B_.size() > DELTA)
				    {
					unsafe_B = intersection_B_;
					avoid_B = true;
				    }
				}
			    }
			    else
			    {
				unsafe_B = B_time;
				avoid_B = true;
			    }
			}
		    }		    
		}		
	    }
	}
	return std::pair<bool, bool>(avoid_A, avoid_B);
    } 
    
    
    
/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    
//    test_CBS_R_1();
//    test_CBS_R_2();
//    test_CBS_R_3();
//    test_CBS_R_4();
//    test_CBS_R_5();
//    test_CBS_R_6();
//    test_CBS_R_7();
//    test_CBS_R_8();
//    test_CBS_R_9();
//    test_CBS_R_A();
//    test_CBS_R_B();
//    test_CBS_R_C();
//    test_CBS_R_D();
//    test_CBS_R_E();
//    test_CBS_R_F();
//    test_CBS_R_G();
//    test_CBS_R_H();
//    test_CBS_R_I();
//    test_CBS_R_J();
//    test_CBS_R_K();
//    test_CBS_R_L();    
    
//    test_CBS_R_E8();
      test_Collision_1();
      test_Collision_1B();      
      test_Collision_2();
      test_Collision_3();
      test_Collision_4();
      test_Collision_5();
      test_Collision_6();
      test_Collision_7();
      test_Collision_8();
      test_Collision_9();
      test_Collision_10();
      test_Collision_11();
      test_Collision_12();
      test_Collision_13();      
}
