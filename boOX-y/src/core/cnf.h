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
/* cnf.h / 1-224_leibniz                                                      */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF formula production tools.
//
/*----------------------------------------------------------------------------*/

#ifndef __CNF_H__
#define __CNF_H__

#include <vector>
#include <list>
#include <map>

#include <errno.h>
#include <signal.h>
#include <zlib.h>
#include <sys/resource.h>

#include "defs.h"
#include "result.h"

#include "common/types.h"

#include "glucose/System.h"
#include "glucose/ParseUtils.h"
#include "glucose/Options.h"
#include "glucose/Dimacs.h"

#include "glucose/Solver.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


    

/*----------------------------------------------------------------------------*/
// sBoolEncoder

    class sBoolEncoder
    {
    public:
	typedef std::vector<sInt_32> VariableIDs_vector;
	typedef VariableIDs_vector VariableIDs_1vector;
	
	typedef std::vector<VariableIDs_1vector> VariableIDs_2vector;
	typedef std::vector<VariableIDs_2vector> VariableIDs_3vector;
	typedef std::vector<VariableIDs_3vector> VariableIDs_4vector;
	typedef std::vector<VariableIDs_4vector> VariableIDs_5vector;
	
    public:
	sBoolEncoder();
	sBoolEncoder(sInt_32 last_variable_ID);

    private:
	sBoolEncoder(const sBoolEncoder &bool_encoder);
	const sBoolEncoder& operator=(const sBoolEncoder &bool_encoder);

    public:
	sInt_32 get_LastVariableID(void) const;	
	void set_LastVariableID(sInt_32 last_variable_ID);
	/*----------------------------------------------------------------*/	

	void cast_AllMutexConstraint(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 weight = 0);	
	void cast_LinearAllMutexConstraint(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 weight = 0);	
	void cast_AdaptiveAllMutexConstraint(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 weight = 0);
	void cast_Disjunction(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 weight = 0);		
	void cast_Mutex(Glucose::Solver *solver, sInt_32 variable_ID_A, sInt_32 variable_ID_B, sInt_32 weight = 0);
	void cast_3Mutex(Glucose::Solver *solver, sInt_32 variable_ID_A, sInt_32 variable_ID_B, sInt_32 variable_ID_C, sInt_32 weight = 0);
	void cast_4Mutex(Glucose::Solver *solver, sInt_32 variable_ID_A, sInt_32 variable_ID_B, sInt_32 variable_ID_C, sInt_32 variable_ID_D, sInt_32 weight = 0);	
	void cast_Mutexes(Glucose::Solver *solver, VariableIDs_vector &variable_IDs_A, VariableIDs_vector &variable_IDs_B, sInt_32 weight = 0);
	void cast_CapacityMutex(Glucose::Solver *solver, VariableIDs_vector &variable_IDs, sInt_32 weight = 0);		
	void cast_ConditionalAllMutexConstraint(Glucose::Solver *solver, sInt_32  &spec_condition, VariableIDs_vector &variable_IDs, sInt_32 weight = 0);	
	void cast_BitSet(Glucose::Solver *solver, sInt_32 variable_ID, sInt_32 weight = 0);	
	void cast_BitUnset(Glucose::Solver *solver, sInt_32 variable_ID, sInt_32 weight = 0);	

	void cast_TriangleMutex(Glucose::Solver *solver,
				sInt_32          variable_ID_A,
				sInt_32          variable_ID_B,
				sInt_32          variable_ID_C,
				sInt_32          weight = 0);

	void cast_MultiTriangleMutex(Glucose::Solver    *solver,
				     sInt_32             variable_ID_A,
				     sInt_32             variable_ID_B,
				     VariableIDs_vector &variable_IDs_C,
				     sInt_32             weight = 0);		

	void cast_BiangleMutex(Glucose::Solver *solver,
			       sInt_32          variable_ID_A,
			       sInt_32          variable_ID_B,
			       sInt_32          weight = 0);

	void cast_MultiBiangleMutex(Glucose::Solver    *solver,
				    sInt_32             variable_ID_A,
				    VariableIDs_vector &variable_IDs_B,
				    sInt_32             weight = 0);		

	void cast_Implication(Glucose::Solver *solver,
			      sInt_32          variable_ID_PREC,
			      sInt_32          variable_ID_POST,
			      sInt_32          weight = 0);	

	void cast_Implication(Glucose::Solver *solver,
			      sInt_32          variable_ID_PREC,
			      sInt_32          variable_ID_POST_A,
			      sInt_32          variable_ID_POST_B,
			      sInt_32          weight = 0);	
	
	void cast_NonzeroImplication(Glucose::Solver *solver,
				     sInt_32          variable_ID_PREC,
				     sInt_32          variable_ID_POST,
				     sInt_32          weight = 0);	

	void cast_ZeroImplication(Glucose::Solver *solver,
				  sInt_32          variable_ID_PREC,
				  sInt_32          variable_ID_POST,
				  sInt_32          weight = 0);	

	void cast_Effect(Glucose::Solver *solver,
			 sInt_32          variable_ID_PREC_A,
			 sInt_32          variable_ID_PREC_B,
			 sInt_32          variable_ID_POST,
			 sInt_32          weight = 0);	

	void cast_MultiImplication(Glucose::Solver    *solver,
				   sInt_32             variable_ID_PREC,
				   VariableIDs_vector &variable_IDs_POST,
				   sInt_32             weight = 0);

	void cast_MultiImpliedImplication(Glucose::Solver    *solver,
					  sInt_32             variable_ID_PREC,
					  VariableIDs_vector &variable_IDs_MIDDLE,
					  VariableIDs_vector &variable_IDs_POST,
					  sInt_32             weight = 0);	

	void cast_MultiConjunctiveImplication(Glucose::Solver    *solver,
					      sInt_32             variable_ID_PREC,
					      VariableIDs_vector &variable_IDs_POST,
					      sInt_32             weight = 0);		
	
	void cast_MultiNegation(Glucose::Solver    *solver,
				VariableIDs_vector &variable_IDs,
				sInt_32             weight = 0);	

	void cast_MultiNegativeImplication(Glucose::Solver    *solver,
					   sInt_32             variable_ID_PREC,
					   VariableIDs_vector &variable_IDs_POST,
					   sInt_32             weight = 0);	

	void cast_MultiExclusiveImplication(Glucose::Solver    *solver,
					    sInt_32             variable_ID_PREC,
					    VariableIDs_vector &variable_IDs_POST,
					    sInt_32             weight = 0);
	
	void cast_SwapConstraint(Glucose::Solver *solver,
				 sInt_32          variable_ID_PREC_A,
				 sInt_32          variable_ID_PREC_B,
				 sInt_32          variable_ID_POST_A,
				 sInt_32          variable_ID_POST_B,
				 sInt_32          weight = 0);	
	
	void cast_NegativeSwapConstraint(Glucose::Solver *solver,
					 sInt_32          variable_ID_PREC_A,
					 sInt_32          variable_ID_PREC_B,
					 sInt_32          variable_ID_POST_A,
					 sInt_32          variable_ID_POST_B,
					 sInt_32          weight = 0);	

	void cast_SwapConstraint(Glucose::Solver    *solver,
				 sInt_32             variable_ID_PREC_A,
				 sInt_32             variable_ID_PREC_B,
				 VariableIDs_vector &variable_IDs_POST,
				 sInt_32             weight = 0);	

	void cast_Cardinality(Glucose::Solver    *solver,
			      VariableIDs_vector &variable_IDs,
			      sInt_32             cardinality,
			      sInt_32             weight = 0);
	/*----------------------------------------------------------------*/
	
	void cast_Clause(Glucose::Solver *solver, sInt_32 lit_1);
	void cast_Clause(Glucose::Solver *solver, sInt_32 lit_1, sInt_32 lit_2);
	void cast_Clause(Glucose::Solver *solver, sInt_32 lit_1, sInt_32 lit_2, sInt_32 lit_3);
	void cast_Clause(Glucose::Solver *solver, sInt_32 lit_1, sInt_32 lit_2, sInt_32 lit_3, sInt_32 lit_4);	
	void cast_Clause(Glucose::Solver *solver, std::vector<sInt_32> &Lits);	

    protected:
	sInt_32 m_last_variable_ID;
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __CNF_H__ */
