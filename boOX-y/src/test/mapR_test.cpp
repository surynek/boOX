/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-213_planck                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                  */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* mapR_test.cpp / 2-213_planck                                               */
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

    
    sResult test_MapR_1(void)
    {
	sResult result;
	printf("MapR test 1 ...\n");

	s2DMap map;
	if (sFAILED(result = map.from_File_movi("../../maps/grid_04x04_obst.map")))
	{
	    printf("Cannot load the map from file (code = %d).\n", result);
	    return result;
	}
	map.to_Screen();
	bool visible_1 = map.check_Visibility(1, 3, 0.3535533905933);	
	bool visible_2 = map.check_Visibility(4, 6, 0.3535533905933);

	printf("Visibility: %d, %d\n", visible_1, visible_2);
	
	printf("MapR test 1 ... finished\n");

	return sRESULT_SUCCESS;
    }


    sResult test_MapR_2(void)
    {
	sResult result;
	printf("MapR test 2 ...\n");

	s2DMap map;
	if (sFAILED(result = map.from_File_movi("../../maps/grid_16x16_obst.map")))
	{
	    printf("Cannot load the map from file (code = %d).\n", result);
	    return result;
	}
	map.to_Screen();
	bool visible_1 = map.check_Visibility(16, 18, 0.3535533905933);	
	bool visible_2 = map.check_Visibility(29, 55, 0.3535533905933);

	printf("Visibility: %d, %d\n", visible_1, visible_2);
	
	printf("MapR test 2 ... finished\n");

	return sRESULT_SUCCESS;
    }


    sResult test_MapR_3(void)
    {
	sResult result;
	printf("MapR test 3 ...\n");

	s2DMap map;
	if (sFAILED(result = map.from_File_movi("../../maps/den520d.map")))
	{
	    printf("Cannot load the map from file (code = %d).\n", result);
	    return result;
	}
	//map.to_Screen();
	//bool visible_1 = map.check_Visibility(16, 18, 0.3535533905933);	
	//bool visible_2 = map.check_Visibility(29, 55, 0.3535533905933);
	
	bool visible_3 = map.check_Visibility(21898, 21899, 0.3535533905933);
	printf("----\n");
	bool visible_4 = map.check_Visibility(5320, 5782, 0.3535533905933);
	printf("----\n");

	map.check_Visibility(21899, 21898, 0.3535533905933);
	printf("----\n");
	map.check_Visibility(5782, 5320, 0.3535533905933);
	printf("----\n");		

	//printf("Visibility: %d, %d\n", visible_1, visible_2);
	printf("Visibility: %d, %d\n", visible_3, visible_4);
	
	printf("MapR test 3 ... finished\n");

	return sRESULT_SUCCESS;
    }                


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    sResult result;
    print_Introduction();
    
    if (sFAILED(result = test_MapR_1()))
    {
	return result;
    }
    if (sFAILED(result = test_MapR_2()))
    {
	return result;
    }
    if (sFAILED(result = test_MapR_3()))
    {
	return result;
    }        

    return 0;
}
