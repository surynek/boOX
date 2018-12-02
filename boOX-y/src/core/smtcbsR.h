/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-161                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                                                                            */
/*          pavel.surynek@fit.cvut.cz | <pavel.surynek@fit.cvut.cz>           */
/*        http://users.fit.cvut.cz/surynek | <http://www.surynek.com>         */
/*                                                                            */
/*============================================================================*/
/* smtcbsR.h / 0_iskra-161                                                    */
/*----------------------------------------------------------------------------*/
//
// Conflict based search for a semi-continuous version of MAPF implemented
// on top of SAT-modulo theories.
//
/*----------------------------------------------------------------------------*/


#ifndef __SMTCBSR_H__
#define __SMTCBSR_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "types.h"
#include "result.h"

#include "core/graph.h"
#include "core/agent.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sRealSmtCBS

    class sRealSMTCBS
    {
    public:
//	sCBS(sInstance *instance);
//	sCBS(sInstance *instance, sDouble timeout);
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __SMTCBSR_H__ */
