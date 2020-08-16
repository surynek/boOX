/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-041_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* agent_test.h / 2-041_planck                                                */
/*----------------------------------------------------------------------------*/
//
// Agent and multi-agent problem related structures - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __AGENT_TEST_H__
#define __AGENT_TEST_H__

#include "core/agent.h"

using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void);
    
    void test_agent_1(const sString &filename);
    void test_agent_2(void);
    void test_agent_3(const sString &filename);

    
/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __AGENT_TEST_H__ */
