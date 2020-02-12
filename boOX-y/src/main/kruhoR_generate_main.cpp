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
/* kruhoR_generate_main.cpp / 1-224_leibniz                                   */
/*----------------------------------------------------------------------------*/
//
// Continuous Multi-Agent Path Finding (MAPF-R) instance (real kruhobot
// configuration, conjunction) generator - main program.
//
// This program takes a continuous map and generated a MAPF-R instance
// on top of that.
//
/*----------------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>
#include <math.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

#include "core/graph.h"
#include "core/mapR.h"
#include "core/kruhoR.h"

#include "util/statistics.h"

#include "main/kruhoR_generate_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_walk(false)
      , m_N_kruhobots(4)
      , m_seed(0)
      , m_kruhobot_radius(0.1)
      , m_kruhobot_linear_velocity(1.0)	
      , m_kruhobot_linear_acceleration(-1.0)
      , m_kruhobot_angular_velocity(-1)
      , m_kruhobot_angular_acceleration(-1)
      , m_kruhobot_wait_factor(0.2)
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("------------------------------------------------------------------------------------\n");
	printf("%s : Continuous Multi-agent Path Finding Kruhobots (kruR) Generator\n", sPRODUCT); 
	printf("%s\n", sCOPYRIGHT);
	printf("====================================================================================\n");	
    }


    void print_ConcludingMessage(void)
    {
	printf("--------------------------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("kruhoR_generate_boOX  --input-mapR-file=<string>\n");
	printf("                      --input-xml-map-file=<string>\n");
	printf("                      --input-xml-agent-file=<string>\n");		
	printf("                      --output-kruhoR-file=<string>\n");
	printf("                      --N-kruhobots=<int>\n");
	printf("                     [--walk]\n");		
	printf("                     [--seed=<int>]\n");
	printf("                     [--kruho-radius=<double>]\n");
	printf("                     [--kruho-velocity=<double>]\n");
	printf("\n");
	printf("Examples:\n");
	printf("kruhoR_generate_boOX --input-mapR-file=ost003d.mapR\n");
	printf("                     --output-kruhoR-file=ost003d.kruR\n");
	printf("                     --N-kruhobots=10\n");	
	printf("                     --kruho-radius=0.2\n");
	printf("\n");
	printf("Defaults: --N-kruhobots=4\n");
	printf("          --kruho-radius=0.1\n");
	printf("          --kruho-velocity=1.0\n");	
	printf("          --seed=0\n");
	printf("\n");
    }


    sResult generate_RealKruhobotInstance(const sCommandParameters &parameters)
    {
	sResult result;
	s2DMap real_Map;
	sRealInstance real_Instance(NULL);

	srand(parameters.m_seed);	

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("GENERATION");
	}
  	#endif

	if (!parameters.m_input_mapR_filename.empty())
	{
	    result = real_Map.from_File_mapR(parameters.m_input_mapR_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open continuous map (mapR) file %s (code = %d).\n", parameters.m_input_mapR_filename.c_str(), result);
		return result;
	    }
	}
	
	if (!parameters.m_input_xml_map_filename.empty() && !parameters.m_input_xml_agent_filename.empty())
	{
	    s2DMap real_Map;

	    result = real_Map.from_File_xml(parameters.m_input_xml_map_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open xml map file %s (code = %d).\n", parameters.m_input_xml_map_filename.c_str(), result);
		return result;
	    }	    		
	    sRealConjunction start_conjunction(&real_Map, 0);    
	    sRealConjunction goal_conjunction(&real_Map, 0);	

	    if (parameters.m_N_kruhobots >= 0)
	    {
		result = start_conjunction.from_File_xml_init(parameters.m_input_xml_agent_filename, parameters.m_N_kruhobots);
	    }
	    else
	    {
		result = start_conjunction.from_File_xml_init(parameters.m_input_xml_agent_filename);
	    }

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open xml initial configuration file %s (code = %d).\n", parameters.m_input_xml_agent_filename.c_str(), result);
		return result;
	    }

	    if (parameters.m_N_kruhobots >= 0)
	    {
		result = goal_conjunction.from_File_xml_goal(parameters.m_input_xml_agent_filename, parameters.m_N_kruhobots);
	    }
	    else
	    {		
		result = goal_conjunction.from_File_xml_goal(parameters.m_input_xml_agent_filename);
	    }

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open xml goal configuration file %s (code = %d).\n", parameters.m_input_xml_agent_filename.c_str(), result);
		return result;
	    }	    
	    real_Instance = sRealInstance(start_conjunction, goal_conjunction);

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= start_conjunction.get_KruhobotCount(); ++kruhobot_id)
	    {
		sKruhobot kruhobot(kruhobot_id, sKruhobot::Properties(parameters.m_kruhobot_radius,
								      parameters.m_kruhobot_linear_velocity,
								      parameters.m_kruhobot_linear_acceleration,
								      parameters.m_kruhobot_angular_velocity,
								      parameters.m_kruhobot_angular_acceleration,
								      parameters.m_kruhobot_wait_factor),
				   sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
		real_Instance.add_Kruhobot(kruhobot_id, kruhobot);
	    }	    
	}
	else
	{
	    sInt_32 N_locations = real_Map.m_Locations.size();	
	
	    sRealConjunction start_conjunction(&real_Map, parameters.m_N_kruhobots);
	    generate_RandomKruhobotConjunction(parameters, N_locations, start_conjunction);
	    
	    sRealConjunction goal_conjunction(&real_Map, parameters.m_N_kruhobots);	
	    if (parameters.m_walk)
	    {
		generate_WalkKruhobotConjunction(parameters, start_conjunction, goal_conjunction);
	    }
	    else
	    {
		generate_RandomKruhobotConjunction(parameters, N_locations, goal_conjunction);
	    }
	    real_Instance = sRealInstance(start_conjunction, goal_conjunction);

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= parameters.m_N_kruhobots; ++kruhobot_id)
	    {
		sKruhobot kruhobot(kruhobot_id, sKruhobot::Properties(parameters.m_kruhobot_radius,
								      parameters.m_kruhobot_linear_velocity,
								      parameters.m_kruhobot_linear_acceleration,
								      parameters.m_kruhobot_angular_velocity,
								      parameters.m_kruhobot_angular_acceleration,
								      parameters.m_kruhobot_wait_factor),
				   sKruhobot::State(M_PI / 4, sKruhobot::Position(0.0, 0.0)));
		real_Instance.add_Kruhobot(kruhobot_id, kruhobot);
	    }
	}

	if (!parameters.m_output_kruhoR_filename.empty())
	{
	    result = real_Instance.to_File_mpfR(parameters.m_output_kruhoR_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write real kruhobot instance (kruR) file %s (code = %d).\n", parameters.m_output_kruhoR_filename.c_str(), result);
		return result;
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.leave_Phase();
	}
	#endif
	
	#ifdef sSTATISTICS
	{
	    s_GlobalStatistics.to_Screen();
	}
	#endif

	return sRESULT_SUCCESS;
    }


    void generate_RandomKruhobotConjunction(const sCommandParameters &parameters, sInt_32 N_locations, sRealConjunction &conjunction)
    {	
	s2DMap::LocationIDs_vector location_IDs;
	for (sInt_32 location_id = 0; location_id < N_locations; ++location_id)
	{
	    location_IDs.push_back(location_id);
	}

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= parameters.m_N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 random_location = rand() % location_IDs.size();
	    sInt_32 random_location_id = location_IDs[random_location];
	    location_IDs[random_location_id] = location_IDs.back();
	    location_IDs.pop_back();

	    conjunction.place_Kruhobot(kruhobot_id, random_location_id);	    
	}	
    }


    void generate_WalkKruhobotConjunction(const sCommandParameters &sUNUSED(parameters), sRealConjunction &sUNUSED(initial_conjunction), sRealConjunction &sUNUSED(conjunction))
    {
	// TODO
	sASSERT(false);
    }    

    
/*----------------------------------------------------------------------------*/
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	if (parameter.find("--input-mapR-file=") == 0)
	{
	    command_parameters.m_input_mapR_filename = parameter.substr(18, parameter.size());
	}
	else if (parameter.find("--input-xml-map-file=") == 0)
	{
	    command_parameters.m_input_xml_map_filename = parameter.substr(21, parameter.size());
	}
	else if (parameter.find("--input-xml-agent-file=") == 0)
	{
	    command_parameters.m_input_xml_agent_filename = parameter.substr(23, parameter.size());
	}		
	else if (parameter.find("--output-kruhoR-file=") == 0)
	{
	    command_parameters.m_output_kruhoR_filename = parameter.substr(21, parameter.size());
	}
	else if (parameter.find("--N-kruhobots=") == 0)
	{
	    command_parameters.m_N_kruhobots = sInt_32_from_String(parameter.substr(14, parameter.size()));
	}
	else if (parameter.find("--seed=") == 0)
	{
	    command_parameters.m_seed = sInt_32_from_String(parameter.substr(7, parameter.size()));
	}
	else if (parameter.find("--walk") == 0)
	{
	    command_parameters.m_walk = true;
	}
	else if (parameter.find("--kruho-radius=") == 0)
	{
	    command_parameters.m_kruhobot_radius = sDouble_from_String(parameter.substr(15, parameter.size()));
	}
	else if (parameter.find("--kruho-velocity=") == 0)
	{
	    command_parameters.m_kruhobot_linear_velocity = sDouble_from_String(parameter.substr(17, parameter.size()));
	}	
	else
	{
	    return sKRUHOBOT_R_GENERATE_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	}
	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int argc, char **argv)
{
    sResult result;
    sCommandParameters command_parameters;

    print_IntroductoryMessage();

    if (argc >= 2 && argc <= 11)
    {
	for (int i = 1; i < argc; ++i)
	{
	    result = parse_CommandLineParameter(argv[i], command_parameters);
	    if (sFAILED(result))
	    {
		printf("Error: Cannot parse command line parameters (code = %d).\n", result);
		print_Help();

		return result;
	    }
	}
	
	result = generate_RealKruhobotInstance(command_parameters);
	if (sFAILED(result))
	{
	    return result;
	}
    }
    else
    {
	print_Help();
    }
    return sRESULT_SUCCESS;
}
