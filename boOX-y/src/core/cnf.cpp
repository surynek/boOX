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
/* cnf.cpp / 1-224_leibniz                                                    */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF formula production tools.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "compile.h"
#include "defs.h"

#include "core/cnf.h"
#include "util/statistics.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sBoolEncoder

    sBoolEncoder::sBoolEncoder()
	: m_last_variable_ID(1)
    {
	// nothing
    }


    sBoolEncoder::sBoolEncoder(sInt_32 last_variable_ID)
	: m_last_variable_ID(last_variable_ID)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/
    
    sInt_32 sBoolEncoder::get_LastVariableID(void) const
    {
	return m_last_variable_ID;
    }

    
    void sBoolEncoder::set_LastVariableID(sInt_32 last_variable_ID)
    {
	m_last_variable_ID = last_variable_ID;
    }
    

/*----------------------------------------------------------------------------*/
 
    void sBoolEncoder::cast_AllMutexConstraint(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 sUNUSED(weight))
    {
	sInt_32 N_Identifiers = variable_IDs.size();
	sInt_32 N_Identifiers_1 = N_Identifiers - 1;

	typedef std::vector<int> CNFs_vector;
	CNFs_vector prec_IDs;
	prec_IDs.resize(N_Identifiers);

	for (sInt_32 id = 0; id < N_Identifiers; ++id)
	{
	    prec_IDs[id] = variable_IDs[id];
	}
	
	for (sInt_32 id_A = 0; id_A < N_Identifiers_1; ++id_A)
	{
	    for (sInt_32 id_B = id_A + 1; id_B < N_Identifiers; ++id_B)
	    {
		cast_Clause(solver, -prec_IDs[id_A], -prec_IDs[id_B]);

                #ifdef sSTATISTICS
		{
		    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
		}
	        #endif		
	    }
	}
    }    


    void sBoolEncoder::cast_LinearAllMutexConstraint(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 sUNUSED(weight))
    {
	sInt_32 N_Identifiers = variable_IDs.size();
	sInt_32 N_Identifiers_1 = N_Identifiers - 1;

	VariableIDs_vector partial_sum_0;
	partial_sum_0.resize(N_Identifiers);

	for (sInt_32 id = 0; id < N_Identifiers; ++id)
	{
	    partial_sum_0[id] = m_last_variable_ID++;
	}

	VariableIDs_vector partial_sum_1;
	partial_sum_1.resize(N_Identifiers);

	for (sInt_32 id = 0; id < N_Identifiers; ++id)
	{
	    partial_sum_1[id] = m_last_variable_ID++;
	}	
   
	typedef std::vector<int> CNFs_vector;
	CNFs_vector prec_IDs;
	prec_IDs.resize(N_Identifiers);

	for (sInt_32 id = 0; id < N_Identifiers; ++id)
	{
	    prec_IDs[id] = variable_IDs[id];
	}
	
	for (sInt_32 id = 0; id < N_Identifiers_1; ++id)
	{
	    cast_Clause(solver, -partial_sum_0[id], partial_sum_0[id + 1]);		
	    cast_Clause(solver, -partial_sum_1[id], partial_sum_1[id + 1]);
	    cast_Clause(solver, -variable_IDs[id], partial_sum_0[id]);;

            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 3;
	    }
 	    #endif	    	    
	}
	for (sInt_32 id = 1; id < N_Identifiers; ++id)
	{
	    cast_Clause(solver, -variable_IDs[id], -partial_sum_0[id - 1], partial_sum_1[id]);

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
 	    #endif
	}	
	cast_Clause(solver, -partial_sum_1[N_Identifiers_1]);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    

    void sBoolEncoder::cast_AdaptiveAllMutexConstraint(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 weight)
    {
//	if (variable_IDs.size() <= 0)		
//	if (variable_IDs.size() <= 4)
//	if (variable_IDs.size() <= 6)
	if (variable_IDs.size() <= 16)	    
//	if (variable_IDs.size() <= 32)
//	if (variable_IDs.size() <= 64)	
//	if (variable_IDs.size() <= 512)
	{
	    cast_AllMutexConstraint(solver, variable_IDs, weight);
	}
	else
	{
	    cast_LinearAllMutexConstraint(solver, variable_IDs, weight);
	}
    }    

    
    void sBoolEncoder::cast_Disjunction(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 sUNUSED(weight))
    {
	sInt_32 N_Identifiers = variable_IDs.size();
	std::vector<int> Literals;
	
	for (sInt_32 id = 0; id < N_Identifiers; ++id)
	{
	    Literals.push_back(variable_IDs[id]);
	}
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    void sBoolEncoder::cast_Mutex(Glucose::Solver *solver, sInt_32 variable_ID_A, sInt_32 variable_ID_B, sInt_32 sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_A, -variable_ID_B);
	
        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif	
    }


    void sBoolEncoder::cast_3Mutex(Glucose::Solver *solver, sInt_32 variable_ID_A, sInt_32 variable_ID_B, sInt_32 variable_ID_C, sInt_32 sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_A, -variable_ID_B, -variable_ID_C);
	
        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif	
    }


    void sBoolEncoder::cast_4Mutex(Glucose::Solver *solver, sInt_32 variable_ID_A, sInt_32 variable_ID_B, sInt_32 variable_ID_C, sInt_32 variable_ID_D, sInt_32 sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_A, -variable_ID_B, -variable_ID_C, -variable_ID_D);
	
        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif	
    }        


    void  sBoolEncoder::cast_Mutexes(Glucose::Solver *solver, VariableIDs_vector &variable_IDs_A, VariableIDs_vector &variable_IDs_B, sInt_32 sUNUSED(weight))	
    {
	for (VariableIDs_vector::const_iterator variable_A = variable_IDs_A.begin(); variable_A != variable_IDs_A.end(); ++variable_A)
	{
	    for (VariableIDs_vector::const_iterator variable_B = variable_IDs_B.begin(); variable_B != variable_IDs_B.end(); ++variable_B)
	    {
		cast_Clause(solver, -*variable_A, -*variable_B);	
	
                #ifdef sSTATISTICS
		{
		    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
		}
      	        #endif
	    }
	}
    }


    void sBoolEncoder::cast_CapacityMutex(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 sUNUSED(weight))
    {
	std::vector<int> Literals;
			    
	for (VariableIDs_vector::const_iterator variable_id = variable_IDs.begin(); variable_id != variable_IDs.end(); ++variable_id)
	{
	    Literals.push_back(-(*variable_id));
	}		
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
      	#endif	
    }


    void sBoolEncoder::cast_ConditionalAllMutexConstraint(Glucose::Solver    *solver,
							  sInt_32            &spec_condition,
							  VariableIDs_vector &variable_IDs,
							  sInt_32             sUNUSED(weight))
    {
	sInt_32 N_Identifiers = variable_IDs.size();
	sInt_32 N_Identifiers_1 = N_Identifiers - 1;
	
	for (sInt_32 id_A = 0; id_A < N_Identifiers_1; ++id_A)
	{
	    for (sInt_32 id_B = id_A + 1; id_B < N_Identifiers; ++id_B)
	    {
		{
		    cast_Clause(solver, -spec_condition, -variable_IDs[id_A], -variable_IDs[id_B]);
		    
                    #ifdef sSTATISTICS
		    {
			++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
		    }
	            #endif		    
		}
	    }
	}
    }    


    void sBoolEncoder::cast_BitSet(Glucose::Solver *solver, sInt_32 variable_ID, sInt_32 sUNUSED(weight))
    {
	cast_Clause(solver, variable_ID);
	
        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    void sBoolEncoder::cast_BitUnset(Glucose::Solver *solver, sInt_32 variable_ID, sInt_32 sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    void sBoolEncoder::cast_TriangleMutex(Glucose::Solver *solver,
					  sInt_32          variable_ID_A,
					  sInt_32          variable_ID_B,
					  sInt_32          variable_ID_C,
					  sInt_32          sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_A, -variable_ID_B, -variable_ID_C);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    void sBoolEncoder::cast_MultiTriangleMutex(Glucose::Solver    *solver,
					       sInt_32             variable_ID_A,
					       sInt_32             variable_ID_B,
					       VariableIDs_vector &variable_IDs_C,
					       sInt_32             sUNUSED(weight))
    {
	for (VariableIDs_vector::const_iterator variable_ID_C = variable_IDs_C.begin(); variable_ID_C != variable_IDs_C.end(); ++variable_ID_C)
	{
	    cast_Clause(solver, -variable_ID_A, -variable_ID_B, -(*variable_ID_C));
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
#endif
	}
    }        


    void sBoolEncoder::cast_BiangleMutex(Glucose::Solver *solver,
					 sInt_32          variable_ID_A,
					 sInt_32          variable_ID_B,
					 sInt_32          sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_A, -variable_ID_B);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    void sBoolEncoder::cast_MultiBiangleMutex(Glucose::Solver    *solver,
					      sInt_32             variable_ID_A,
					      VariableIDs_vector &variable_IDs_B,
					      sInt_32             sUNUSED(weight))
    {
	for (VariableIDs_vector::const_iterator variable_ID_B = variable_IDs_B.begin(); variable_ID_B != variable_IDs_B.end(); ++variable_ID_B)
	{
	    cast_Clause(solver, -variable_ID_A, -(*variable_ID_B));
		
#ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
#endif
	}
    }        


    void sBoolEncoder::cast_Implication(Glucose::Solver *solver,
					sInt_32          variable_ID_PREC,
					sInt_32          variable_ID_POST,
					sInt_32          sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_PREC, variable_ID_POST);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    void sBoolEncoder::cast_Implication(Glucose::Solver *solver,
					sInt_32          variable_ID_PREC,
					sInt_32          variable_ID_POST_A,
					sInt_32          variable_ID_POST_B,
					sInt_32          sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_PREC, variable_ID_POST_A);
	cast_Clause(solver, -variable_ID_PREC, variable_ID_POST_B);

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }    


    void sBoolEncoder::cast_Effect(Glucose::Solver *solver,
				   sInt_32          variable_ID_PREC_A,
				   sInt_32          variable_ID_PREC_B,
				   sInt_32          variable_ID_POST,
				   sInt_32          sUNUSED(weight))
    {

	cast_Clause(solver, -variable_ID_PREC_A, -variable_ID_PREC_B, variable_ID_POST);

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }    


    void sBoolEncoder::cast_MultiImplication(Glucose::Solver    *solver,
					     sInt_32             variable_ID_PREC,
					     VariableIDs_vector &variable_IDs_POST,
					     sInt_32             sUNUSED(weight))
    {
	std::vector<int> Literals;
	Literals.push_back(-variable_ID_PREC);

	for (VariableIDs_vector::const_iterator variable_ID_POST = variable_IDs_POST.begin(); variable_ID_POST != variable_IDs_POST.end(); ++variable_ID_POST)
	{	    
	    Literals.push_back(*variable_ID_POST);
	}
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    void sBoolEncoder::cast_MultiImpliedImplication(Glucose::Solver    *solver,
						    sInt_32             variable_ID_PREC,
						    VariableIDs_vector &variable_IDs_MIDDLE,
						    VariableIDs_vector &variable_IDs_POST,
						    sInt_32             sUNUSED(weight))
    {
	VariableIDs_vector::const_iterator variable_ID_MIDDLE = variable_IDs_MIDDLE.begin(); 	
	for (VariableIDs_vector::const_iterator variable_ID_POST = variable_IDs_POST.begin(); variable_ID_POST != variable_IDs_POST.end(); ++variable_ID_POST)
	{
	    std::vector<int> Literals;
		
	    Literals.push_back(-variable_ID_PREC);
	    Literals.push_back(-(*variable_ID_MIDDLE));
	    Literals.push_back(*variable_ID_POST);
	    cast_Clause(solver, Literals);

	    ++variable_ID_MIDDLE;

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
  	    #endif	    
	}
    }    


    void sBoolEncoder::cast_MultiConjunctiveImplication(Glucose::Solver    *solver,
							sInt_32             variable_ID_PREC,
							VariableIDs_vector &variable_IDs_POST,
							sInt_32             sUNUSED(weight))
    {
	for (VariableIDs_vector::const_iterator variable_ID_POST = variable_IDs_POST.begin(); variable_ID_POST != variable_IDs_POST.end(); ++variable_ID_POST)
	{
	    cast_Clause(solver, -variable_ID_PREC, *variable_ID_POST);
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif
	}
    }        


    void sBoolEncoder::cast_MultiNegation(Glucose::Solver *solver,
				      VariableIDs_vector  &variable_IDs,
				      sInt_32             sUNUSED(weight))
    {
	for (VariableIDs_vector::const_iterator variable_ID = variable_IDs.begin(); variable_ID != variable_IDs.end(); ++variable_ID)
	{
	    {
		cast_Clause(solver, -(*variable_ID));
	    }

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif
	}
    }    

    
    void sBoolEncoder::cast_MultiNegativeImplication(Glucose::Solver    *solver,
						     sInt_32             variable_ID_PREC,
						     VariableIDs_vector &variable_IDs_POST,
						     sInt_32             sUNUSED(weight))
    {
	for (VariableIDs_vector::const_iterator variable_ID_POST = variable_IDs_POST.begin(); variable_ID_POST != variable_IDs_POST.end(); ++variable_ID_POST)
	{
	    cast_Clause(solver, -variable_ID_PREC, -(*variable_ID_POST));

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
            #endif
	}
    }    


    void sBoolEncoder::cast_MultiExclusiveImplication(Glucose::Solver    *solver,
						      sInt_32             variable_ID_PREC,
						      VariableIDs_vector &variable_IDs_POST,
						      sInt_32             sUNUSED(weight))
    {
	for (VariableIDs_vector::const_iterator variable_ID_POST = variable_IDs_POST.begin(); variable_ID_POST != variable_IDs_POST.end(); ++variable_ID_POST)
	{
	    cast_Clause(solver, variable_ID_PREC, -(*variable_ID_POST));

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
            #endif
	}
    }    


    void sBoolEncoder::cast_SwapConstraint(Glucose::Solver *solver,
					   sInt_32          variable_ID_PREC_A,
					   sInt_32          variable_ID_PREC_B,
					   sInt_32          variable_ID_POST_A,
					   sInt_32          variable_ID_POST_B,
					   sInt_32          sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_PREC_A, -variable_ID_PREC_B, -variable_ID_POST_A);
	cast_Clause(solver, -variable_ID_PREC_A, -variable_ID_PREC_B, -variable_ID_POST_B);

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }

    
    void sBoolEncoder::cast_NegativeSwapConstraint(Glucose::Solver *solver,
						   sInt_32          variable_ID_PREC_A,
						   sInt_32          variable_ID_PREC_B,
						   sInt_32          variable_ID_POST_A,
						   sInt_32          variable_ID_POST_B,
						   sInt_32          sUNUSED(weight))
    {
	cast_Clause(solver, -variable_ID_PREC_A, -variable_ID_PREC_B, variable_ID_POST_A);
	cast_Clause(solver, -variable_ID_PREC_A, -variable_ID_PREC_B, variable_ID_POST_B);

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }    


    void sBoolEncoder::cast_SwapConstraint(Glucose::Solver    *solver,
					   sInt_32             variable_ID_PREC_A,
					   sInt_32             variable_ID_PREC_B,
					   VariableIDs_vector &variable_IDs_POST,
					   sInt_32             sUNUSED(weight))
    {
	sInt_32 N_Identifiers = variable_IDs_POST.size();

	for (VariableIDs_vector::const_iterator variable_ID_POST = variable_IDs_POST.begin(); variable_ID_POST != variable_IDs_POST.end(); ++variable_ID_POST)
	{
	    cast_Clause(solver, -variable_ID_PREC_A, -variable_ID_PREC_B, -(*variable_ID_POST));
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.get_CurrentPhase().m_produced_cnf_Clauses += N_Identifiers;
	}
	#endif
    }    


    void sBoolEncoder::cast_Cardinality(Glucose::Solver    *solver,
					VariableIDs_vector &variable_IDs,
					sInt_32             cardinality,
					sInt_32             sUNUSED(weight))
    {
	 VariableIDs_2vector partial_sum_auxiliary;
	 partial_sum_auxiliary.resize(variable_IDs.size());
	 
	 for (sInt_32 i = 0; i < variable_IDs.size(); ++i)
	 {
	     partial_sum_auxiliary[i].resize(variable_IDs.size());
	 }

	 for (sInt_32 i = 0; i < variable_IDs.size(); ++i)
	 {
	     for (sInt_32 j = 0; j < variable_IDs.size(); ++j)
	     {
		 partial_sum_auxiliary[i][j] = m_last_variable_ID++;
	     }	     
	 }
	 cast_Clause(solver, -variable_IDs[0], partial_sum_auxiliary[0][0]);
	 /*
	 fprintf(fw, "-%d %d 0\n",
		 variable_IDs[0], 
		 partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(0)));
	 */

	 for (sInt_32 i = 1; i < variable_IDs.size(); ++i)
	 {
	     for (sInt_32 j = 0; j <= i; ++j)
	     {
		 cast_Clause(solver, -partial_sum_auxiliary[i-1][j], partial_sum_auxiliary[i][j]);
//		 fprintf(fw, "-%d %d 0\n", partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j)), partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
	     }
	     cast_Clause(solver, -variable_IDs[i], partial_sum_auxiliary[i][0]);
/*	     
	     fprintf(fw, "-%d %d 0\n",
		     variable_IDs[i], 
		     partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(0)));
*/
	     for (sInt_32 j = 1; j <= i; ++j)
	     {
		 cast_Clause(solver, -variable_IDs[i], -partial_sum_auxiliary[i-1][j-1], partial_sum_auxiliary[i][j]);
/*		 
		 fprintf(fw, "-%d -%d %d 0\n",
			 variable_IDs[i], 
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j-1)),
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
*/
	     }
	 }
	 for (sInt_32 i = 0; i < variable_IDs.size(); ++i)
	 {
	     for (sInt_32 j = cardinality; j < variable_IDs.size(); ++j)
	     {
		 cast_Clause(solver, -partial_sum_auxiliary[i][j]);
/*		 
		 fprintf(fw, "-%d 0\n",
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
*/
	     }
	 }
    }   


/*----------------------------------------------------------------------------*/
    
    void sBoolEncoder::cast_Clause(Glucose::Solver *solver, sInt_32 lit_1)
    {
	std::vector<int> Lits;
	Lits.push_back(lit_1);
	
	cast_Clause(solver, Lits);
    }

    
    void sBoolEncoder::cast_Clause(Glucose::Solver *solver, sInt_32 lit_1, sInt_32 lit_2)
    {
	std::vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	
	cast_Clause(solver, Lits);
    }

    
    void sBoolEncoder::cast_Clause(Glucose::Solver *solver, sInt_32 lit_1, sInt_32 lit_2, sInt_32 lit_3)
    {
	std::vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	Lits.push_back(lit_3);
		
	cast_Clause(solver, Lits);
    }


    void sBoolEncoder::cast_Clause(Glucose::Solver *solver, sInt_32 lit_1, sInt_32 lit_2, sInt_32 lit_3,sInt_32 lit_4)
    {
	std::vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	Lits.push_back(lit_3);
	Lits.push_back(lit_4);	
		
	cast_Clause(solver, Lits);
    }    

    
    void sBoolEncoder::cast_Clause(Glucose::Solver *solver, std::vector<int> &Lits)
    {
	Glucose::vec<Glucose::Lit> glu_Lits;
	
	for (std::vector<int>::const_iterator lit = Lits.begin(); lit != Lits.end(); ++lit)
	{
	    sInt_32 glu_var = sABS(*lit) - 1;
	    while (glu_var >= solver->nVars())
	    {
		solver->newVar();
	    }
	    glu_Lits.push((*lit > 0) ? Glucose::mkLit(glu_var, false) : ~Glucose::mkLit(glu_var, false));
	}
	solver->addClause(glu_Lits);
    }

    
/*----------------------------------------------------------------------------*/

} // namespace boOX
