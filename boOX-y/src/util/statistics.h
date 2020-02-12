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
/* statistics.h / 1-224_leibniz                                               */
/*----------------------------------------------------------------------------*/
//
// Statistical data collection and analytical tools.
//
/*----------------------------------------------------------------------------*/


#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include <map>

#include "result.h"

#include "common/types.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{



    
/*----------------------------------------------------------------------------*/
// sStatistics

    class sStatistics
    {
    public:
	static const sDouble SECONDS_UNDEFINED;
	static const sString ROOT_PHASE_NAME;

    public:
	struct Phase;
	typedef std::map<sString, Phase*, std::less<sString> > Phases_map;

	struct Phase
	{
	    Phase(const sString &name, Phase *parent_phase);

	    sString m_name;

	    sDouble m_WC_Seconds;
	    sDouble m_CPU_Seconds;

	    sUInt_64 m_search_Steps;
	    sUInt_64 m_move_Executions;

	    sUInt_64 m_produced_cnf_Clauses;
	    sUInt_64 m_satisfiable_SAT_solver_Calls;
	    sUInt_64 m_unsatisfiable_SAT_solver_Calls;
	    sUInt_64 m_indeterminate_SAT_solver_Calls;	    

	    Phase *m_parent_phase;
	    Phases_map m_sub_Phases;
	};

    private: /* the object cannot be copied */
	sStatistics(const sStatistics &phase_statistics);
	const sStatistics& operator=(const sStatistics &phase_statistics);

    public:
	sStatistics();
	virtual ~sStatistics();
	/*--------------------------------*/

	Phase& get_CurrentPhase(void);
	
	void enter_Root(void);
	void enter_Phase(const sString &phase_name);
	void leave_Phase(void);

	void restart_CurrentPhase(void);
	void suspend_CurrentPhase(void);

	static sDouble get_WC_Seconds(void);
	static sDouble get_CPU_Seconds(void);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	
	virtual void to_Stream(FILE *output, const sString &indent = "") const;
	virtual void to_Stream_subphases(FILE *output, const Phase &phase, const sString &indent = "") const;

    public:
	Phase *m_current_phase;

	sDouble m_curr_phase_start_WC;
	sDouble m_curr_phase_finish_WC;

	sDouble m_curr_phase_start_CPU;
	sDouble m_curr_phase_finish_CPU;

	Phase *m_root_phase;
    };


/*----------------------------------------------------------------------------*/
// Global objects

    extern sStatistics s_GlobalStatistics;


/*----------------------------------------------------------------------------*/
// Global functions

    sDouble sGet_WC_Seconds(void);
    sDouble sGet_CPU_Seconds(void);


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __STATISTICS_H__ */
