/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-214_planck                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                  */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* satnext.h / 2-214_planck                                                   */
/*----------------------------------------------------------------------------*/
//
// Next generation SAT-based algorithms for MAPF and related problems.
//
/*----------------------------------------------------------------------------*/


#ifndef __SATNEXT_H__
#define __SATNEXT_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>

#include "glucose/System.h"
#include "glucose/ParseUtils.h"
#include "glucose/Options.h"
#include "glucose/Dimacs.h"
#include "glucose/Solver.h"

#include "result.h"

#include "common/types.h"

#include "core/graph.h"
#include "core/cbs.h"
#include "core/cnf.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{

/*----------------------------------------------------------------------------*/
// sSATNext
    
    class sSATNext
    {
    public:
	typedef std::vector<sConfiguration> Configurations_vector;
	
    public:
	sSATNext(sBoolEncoder *solver_Encoder, sInstance *instance);

	/*----------------------------------------------------------------------------*/	
	
    private:
	sSATNext(const sSATNext &sat_next);
	const sSATNext& operator=(const sSATNext &sat_next);

    public:		
};

    
/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __SATNEXT_H__ */
