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
/* smtcbsR_test.cpp / 1-224_leibniz                                           */
/*----------------------------------------------------------------------------*/
//
// Test of semi-continuous version of conflict-based search implemented
// in the satisfiability modulo theories framework.
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
#include "core/smtcbsR.h"

#include "util/statistics.h"

#include "test/smtcbsR_test.h"


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
	printf("%s : Test of Semi-continuous SMT/CBS\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");
    }

   
    void test_SMTCBS_R_1(void)
    {
	printf("SMT-CBS-R test 1 ...\n");	

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

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance, -1.0);
	
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);
	
//	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
//	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);
//	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);

	printf("SMT-CBS-R test 1 ... finished\n");
    }


    void test_SMTCBS_R_2(void)
    {
	printf("SMT-CBS-R test 2 ...\n");	

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

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);

	/*
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	*/
	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;
	
	sRealSMTCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	
/*
	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);       
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);
*/

	printf("SMT-CBS-R test 2 ... finished\n");
    }    


    void test_SMTCBS_R_3(void)
    {
	printf("SMT-CBS-R test 3 ...\n");	

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

	sRealConjunction start_conjunction(&crossing_map, 2);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);

	sRealConjunction goal_conjunction(&crossing_map, 2);
	goal_conjunction.place_Kruhobot(1, 3);
	goal_conjunction.place_Kruhobot(2, 2);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);
	
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	
/*
	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);       
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);
*/

	printf("SMT-CBS-R test 3 ... finished\n");
    }        


    void test_SMTCBS_R_4(void)
    {
	printf("SMT-CBS-R test 4 ...\n");	

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

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);

	/*
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	*/
	sRealSMTCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;	
	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	
/*
	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);       
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);
*/

	printf("SMT-CBS-R test 4 ... finished\n");
    }


    void test_SMTCBS_R_5(void)
    {
	printf("SMT-CBS-R test 5 ...\n");

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

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);

	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;	

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	sRealSMTCBS::KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(start_conjunction.get_KruhobotCount() + 1);

	sRealSMTCBS::KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(start_conjunction.get_KruhobotCount() + 1);	    	

	sRealSMTCBS::Traversal traversal_1(1, 0, 5, sRealSMTCBS::Interval(0.4, 0.8));
	real_SMTCBS.introduce_KruhobotConflict(traversal_1, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, false);

	sRealSMTCBS::Traversal traversal_1A(1, 0, 5, sRealSMTCBS::Interval(0.4, 0.8));
	real_SMTCBS.introduce_KruhobotConflict(traversal_1A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, false);

	sRealSMTCBS::Traversal traversal_1B(1, 0, 5, sRealSMTCBS::Interval(0.4, 0.8));
	real_SMTCBS.introduce_KruhobotConflict(traversal_1B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, false);

	sRealSMTCBS::Traversal traversal_2(1, 0, 5, sRealSMTCBS::Interval(1.2, 1.3));
	real_SMTCBS.introduce_KruhobotConflict(traversal_2, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, false);

	sRealSMTCBS::Traversal traversal_3(2, 1, 1, sRealSMTCBS::Interval(0.9, 1.5));
	real_SMTCBS.introduce_KruhobotConflict(traversal_3, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, false);	

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= start_conjunction.get_KruhobotCount(); ++kruhobot_id)
	{
	    real_SMTCBS.build_KruhobotRealDecisionDiagram(instance.m_Kruhobots[kruhobot_id],
							  *instance.m_start_conjunction.m_Map,
							  instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
							  instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
							  kruhobot_location_Conflicts[kruhobot_id],
							  kruhobot_linear_Conflicts[kruhobot_id],
							  2.0,
							  kruhobot_RDDs[kruhobot_id],
							  kruhobot_RDD_Mappings[kruhobot_id]);	
	    real_SMTCBS.to_Screen(kruhobot_RDDs[kruhobot_id]);
	}

	printf("SMT-CBS-R test 5 ... finished\n");
    }


    void test_SMTCBS_R_6(void)
    {
	printf("SMT-CBS-R test 6 ...\n");	

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
	goal_conjunction.place_Kruhobot(1, 7);
	goal_conjunction.place_Kruhobot(2, 6);
	goal_conjunction.place_Kruhobot(3, 5);
	goal_conjunction.place_Kruhobot(4, 4);			

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);
	instance.add_Kruhobot(4, kruhobot_4);		
	instance.to_Screen();

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);
/*	
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
*/
	sRealSMTCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;

	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	
/*
	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);       
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);
*/

	printf("SMT-CBS-R test 6 ... finished\n");
    }


    void test_SMTCBS_R_7(void)
    {
	printf("SMT-CBS-R test 7 ...\n");	

	s2DMap grid_map(9);

	grid_map.add_Location(0, 0.0, 0.0);
	grid_map.add_Location(1, 1.0, 0.0);
	grid_map.add_Location(2, 2.0, 0.0);

	grid_map.add_Location(3, 0.0, 1.0);
	grid_map.add_Location(4, 1.0, 1.0);
	grid_map.add_Location(5, 2.0, 1.0);

	grid_map.add_Location(6, 0.0, 2.0);
	grid_map.add_Location(7, 1.0, 2.0);
	grid_map.add_Location(8, 2.0, 2.0);	       
	    
	grid_map.calc_AllPairsStraightDistances();
	grid_map.populate_Network(1.5);	
	grid_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_1.to_Screen();

	sKruhobot kruhobot_2(2, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_2.to_Screen();

	sKruhobot kruhobot_3(3, sKruhobot::Properties(0.1, 1.0, 1.0, M_PI / 4, M_PI / 6, s_wait_factor), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	kruhobot_3.to_Screen();

	sRealConjunction start_conjunction(&grid_map, 3);
	start_conjunction.place_Kruhobot(1, 0);
	start_conjunction.place_Kruhobot(2, 1);
	start_conjunction.place_Kruhobot(3, 2);

	sRealConjunction goal_conjunction(&grid_map, 3);
	goal_conjunction.place_Kruhobot(1, 7);
	goal_conjunction.place_Kruhobot(2, 6);
	goal_conjunction.place_Kruhobot(3, 8);

	start_conjunction.to_Screen();
	goal_conjunction.to_Screen();

	sRealInstance instance(start_conjunction, goal_conjunction);

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.add_Kruhobot(3, kruhobot_3);
	instance.to_Screen();

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);
/*	
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
*/
	sRealSMTCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;

	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
					      kruhobot_location_Conflicts,
					      kruhobot_linear_Conflicts,
					      kruhobot_Schedules,
					      100.0,
					      10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	
/*
	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);       
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);
*/

	printf("SMT-CBS-R test 7 ... finished\n");
    }        
    


    void test_SMTCBS_R_8(void)
    {
	printf("SMT-CBS-R test 8 ...\n");	

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

	sBoolEncoder encoder;
	sRealSMTCBS real_SMTCBS(&encoder, &instance);
/*	
	sRealSMTCBS::KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;
*/
	sRealSMTCBS::KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	sRealSMTCBS::KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;

	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;

	kruhobot_location_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_linear_Conflicts.resize(start_conjunction.get_KruhobotCount() + 1);
	kruhobot_Schedules.resize(start_conjunction.get_KruhobotCount() + 1);

	real_SMTCBS.find_NonconflictingSchedules(instance,
						 kruhobot_location_Conflicts,
						 kruhobot_linear_Conflicts,
						 kruhobot_Schedules,
						 100.0,
						 10.0);
	sRealCBSBase::to_Screen(kruhobot_Schedules);	
/*
	sDouble solution_cost = sRealCBS::calc_ScheduleCost(instance, kruhobot_Schedules);
	sDouble solution_makespan = sRealCBS::calc_ScheduleMakespan(instance, kruhobot_Schedules);       
	printf("Found solution: cost = %.3f, makespan = %.3f\n", solution_cost, solution_makespan);
*/

	printf("SMT-CBS-R test 8 ... finished\n");
    }        

    
    
    
/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    
//    test_SMTCBS_R_1();
//    test_SMTCBS_R_2();
//    test_SMTCBS_R_3();
//    test_SMTCBS_R_4();
//    test_SMTCBS_R_5();
//    test_SMTCBS_R_6();
//    test_SMTCBS_R_7();
    test_SMTCBS_R_8();        
}
