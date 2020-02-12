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
/* kruhoR_test.cpp / 1-224_leibniz                                            */
/*----------------------------------------------------------------------------*/
//
// Semi-continuous MAPF (MAPR-R), Kurhobots and related structures.
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

#include "util/statistics.h"

#include "test/kruhoR_test.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void)
    {
	printf("----------------------------------------------------------------\n");
	printf("%s : Semi-continuous Kruhobot Test Program\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");
    }

    
    void test_KruhobotR_1(void)
    {
	printf("KruhobotR test 1 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.to_Screen();

	printf("KruhobotR test 1 ... finished\n");
    }


    void test_KruhobotR_2(void)
    {
	printf("KruhobotR test 2 ...\n");
	
	s2DMap square_map(4);

	square_map.add_Location(0, 0.0, 0.0);
	square_map.add_Location(1, 1.0, 0.0);
	square_map.add_Location(2, 0.0, 1.0);
	square_map.add_Location(3, 1.0, 1.0);

	square_map.calc_AllPairsStraightDistances();
	square_map.to_Screen();

	sKruhobot kruhobot_1(1, sKruhobot::Properties(1.0, 2.0, 1.0, M_PI / 4, M_PI / 6, 0.2), sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
	sKruhobot kruhobot_2(2, sKruhobot::Properties(2.0, 3.0, 2.0, M_PI / 6, M_PI / 4, 0.2), sKruhobot::State(M_PI / 3, sKruhobot::Position(1.0, 0.0)));

	kruhobot_1.to_Screen();
	kruhobot_2.to_Screen();

	sRealConjunction conjunction_1(&square_map, 2);
	conjunction_1.place_Kruhobot(1, 0);
	conjunction_1.place_Kruhobot(2, 1);

	sRealConjunction conjunction_2(&square_map, 2);
	conjunction_2.place_Kruhobot(1, 3);
	conjunction_2.place_Kruhobot(2, 0);

	conjunction_1.to_Screen();
	conjunction_2.to_Screen();

	sRealInstance instance(conjunction_1, conjunction_2);
	instance.to_Screen();

	instance.add_Kruhobot(1, kruhobot_1);
	instance.add_Kruhobot(2, kruhobot_2);
	instance.to_Screen();	

	printf("KruhobotR test 2 ... finished\n");
    }


    void test_KruhobotR_3(void)
    {
	printf("KruhobotR test 3 ...\n");

	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 1.1);
	    map.add_Location(1, 2.0, 4.0);
	    
	    map.add_Location(2, 1.5, 3.0);
	    map.add_Location(3, 5.0, 3.0);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}
	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 1.1);
	    map.add_Location(1, 2.0, 4.0);
	    
	    map.add_Location(2, 1.5, 1.0);
	    map.add_Location(3, 5.0, 2.0);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}
	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 1.1);
	    map.add_Location(1, 2.0, 4.0);
	    
	    map.add_Location(2, 3.0, 2.0);
	    map.add_Location(3, 4.0, 4.0);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}
	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 3.0);
	    map.add_Location(1, 1.0, 4.0);
	    
	    map.add_Location(2, 2.0, 1.0);
	    map.add_Location(3, 4.0, 1.0);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}
	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 3.0);
	    map.add_Location(1, 1.0, 4.0);
	    
	    map.add_Location(2, 2.0, 1.0);
	    map.add_Location(3, 4.0, 2.0);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}
	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 3.0);
	    map.add_Location(1, 1.0, 4.0);
	    
	    map.add_Location(2, 2.0, 1.0);
	    map.add_Location(3, 4.0, 3.0);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}
	{
	    s2DMap map(4);

	    map.add_Location(0, 1.0, 3.0);
	    map.add_Location(1, 1.0, 4.0);
	    
	    map.add_Location(2, 2.0, 1.0);
	    map.add_Location(3, 2.5, 2.5);

	    sDouble line_distance_1 = map.calc_LineDistance(0, 1, 2, 3);
	    printf("Line distance: %.3f\n", line_distance_1);

	    sDouble line_distance_2 = map.calc_LineDistance(2, 3, 0, 1);
	    printf("Line distance opposite: %.3f\n", line_distance_2);
	}						

	printf("KruhobotR test 3 ... finished\n");
    }        


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    print_Introduction();
    
    test_KruhobotR_1();
    test_KruhobotR_2();
    test_KruhobotR_3();    
}
