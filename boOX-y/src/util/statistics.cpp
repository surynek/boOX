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
/* statistics.cpp / 1-224_leibniz                                             */
/*----------------------------------------------------------------------------*/
//
// Statistical data collection and analytical tools.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/times.h>

#include "config.h"
#include "compile.h"

#include "util/statistics.h"


using namespace boOX;




/*----------------------------------------------------------------------------*/

namespace boOX
{

    

    
/*============================================================================*/
// sStatistics class
/*----------------------------------------------------------------------------*/

    const double sStatistics::SECONDS_UNDEFINED = 1.0;
    const sString sStatistics::ROOT_PHASE_NAME = "root_phase";


/*----------------------------------------------------------------------------*/

    sStatistics::Phase::Phase(const sString &name, Phase *parent_phase)
	: m_name(name)
	, m_WC_Seconds(0.0)
	, m_CPU_Seconds(0.0)
	, m_search_Steps(0)
	, m_move_Executions(0)
	, m_produced_cnf_Clauses(0)
	, m_satisfiable_SAT_solver_Calls(0)
	, m_unsatisfiable_SAT_solver_Calls(0)
	, m_indeterminate_SAT_solver_Calls(0)	  
	, m_parent_phase(parent_phase)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sStatistics::sStatistics()
    {
	m_root_phase = new Phase(ROOT_PHASE_NAME, NULL);
	m_current_phase = m_root_phase;
	restart_CurrentPhase();
    }


    sStatistics::~sStatistics()
    {
	for (Phases_map::iterator phase = m_root_phase->m_sub_Phases.begin(); phase != m_root_phase->m_sub_Phases.end(); ++phase)
	{
	    delete phase->second;
	}
	delete m_root_phase;
    }


/*----------------------------------------------------------------------------*/
// sStatistics public methods
    
    sStatistics::Phase& sStatistics::get_CurrentPhase(void)
    {
	return *m_current_phase;
    }


    void sStatistics::enter_Root(void)
    {
	while (m_current_phase->m_parent_phase != NULL)
	{
	    leave_Phase();
	}
    }


    void sStatistics::enter_Phase(const sString &phase_name)
    {
	if (phase_name != m_current_phase->m_name)
	{
	    suspend_CurrentPhase();

	    Phases_map::iterator phase = m_current_phase->m_sub_Phases.find(phase_name);
	    if (phase != m_current_phase->m_sub_Phases.end())
	    {
		m_current_phase = phase->second;
	    }
	    else
	    {
		Phase *created_phase = new Phase(phase_name, m_current_phase);
		std::pair<Phases_map::iterator, bool> phase = m_current_phase->m_sub_Phases.insert(Phases_map::value_type(phase_name, created_phase));
		m_current_phase = phase.first->second;
	    }

	    restart_CurrentPhase();
	}
    }


    void sStatistics::leave_Phase(void)
    {
	suspend_CurrentPhase();
	    
	if (m_current_phase->m_parent_phase != NULL)
	{
	    m_current_phase = m_current_phase->m_parent_phase;
	}

	restart_CurrentPhase();
    }


    sDouble sStatistics::get_WC_Seconds(void)
    {
	struct timeval timeval;
	struct timezone timezone;

	if (gettimeofday(&timeval, &timezone) == -1)
	{
	    return SECONDS_UNDEFINED;
	}
	else
	{
	    return timeval.tv_sec + timeval.tv_usec / static_cast<sDouble>(1000000.0);
	}
    }


    void sStatistics::restart_CurrentPhase(void)
    {
	m_curr_phase_start_WC = get_WC_Seconds();
	m_curr_phase_start_CPU = get_CPU_Seconds();
    }


    void sStatistics::suspend_CurrentPhase(void)
    {
	m_curr_phase_finish_WC = get_WC_Seconds();
	m_curr_phase_finish_CPU = get_CPU_Seconds();

	m_current_phase->m_WC_Seconds += m_curr_phase_finish_WC - m_curr_phase_start_WC;
	m_current_phase->m_CPU_Seconds += m_curr_phase_finish_CPU - m_curr_phase_start_CPU;
    }


    sDouble sStatistics::get_CPU_Seconds(void)
    {
	struct tms tms_record;

	if (times(&tms_record) == -1)
	{
	    return SECONDS_UNDEFINED;
	}
	else
	{
	    return ((tms_record.tms_utime + tms_record.tms_stime + tms_record.tms_cutime + tms_record.tms_cstime) / (sDouble)sysconf(_SC_CLK_TCK));
	}
    }


/*----------------------------------------------------------------------------*/

    void sStatistics::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }
    
    
    void sStatistics::to_Stream(FILE *output, const sString &indent) const
    {	
	fprintf(output, "%sPhase statistics (current phase = '%s') [\n", indent.c_str(), m_current_phase->m_name.c_str());
	to_Stream_subphases(output, *m_root_phase, indent + s_INDENT);
	fprintf(output, "%s]\n", indent.c_str());
    }

   
    void sStatistics::to_Stream_subphases(FILE *output, const Phase &phase, const sString &indent) const
    {
	fprintf(output, "%s%sPhase (name = '%s') [\n", indent.c_str(), s_INDENT.c_str(), phase.m_name.c_str());
	fprintf(output, "%s%s%sSearch steps                   = %lld\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_search_Steps);
	fprintf(output, "%s%s%sMove executions                = %lld\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_move_Executions);
	fprintf(output, "%s%s%sProduced cnf clauses           = %lld\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_produced_cnf_Clauses);
	fprintf(output, "%s%s%sSatisfiable SAT solver calls   = %lld\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_satisfiable_SAT_solver_Calls);
	fprintf(output, "%s%s%sUnsatisfiable SAT solver calls = %lld\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_unsatisfiable_SAT_solver_Calls);
	fprintf(output, "%s%s%sIndeterminate SAT solver calls = %lld\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_indeterminate_SAT_solver_Calls);
	
	fprintf(output, "%s%s%sWall clock TIME (seconds)      = %.3f\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_WC_Seconds);
	fprintf(output, "%s%s%sCPU/machine TIME (seconds)     = %.3f\n", indent.c_str(), s_INDENT.c_str(), s_INDENT.c_str(), phase.m_CPU_Seconds);
	fprintf(output, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

	if (!phase.m_sub_Phases.empty())
	{
	    fprintf(output, "%s%sSub-phases {\n", indent.c_str(), s_INDENT.c_str());
	    for (Phases_map::const_iterator sub_phase = phase.m_sub_Phases.begin(); sub_phase != phase.m_sub_Phases.end(); ++sub_phase)
	    {
		to_Stream_subphases(output, *sub_phase->second, indent + s_INDENT);
	    }
	    fprintf(output, "%s%s}\n", indent.c_str(), s_INDENT.c_str());
	}
    }



    
/*============================================================================*/
// Global objects
/*----------------------------------------------------------------------------*/

    sStatistics s_GlobalStatistics;


/*============================================================================*/
// Global functions
/*----------------------------------------------------------------------------*/

    sDouble sGet_WC_Seconds(void)
    {
	return sStatistics::get_WC_Seconds();
    }


    sDouble sGet_CPU_Seconds(void)
    {
	return sStatistics::get_CPU_Seconds();
    }


/*----------------------------------------------------------------------------*/

} // namespace boOX
