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
/* smtcbsR_test.h / 1-224_leibniz                                             */
/*----------------------------------------------------------------------------*/
//
// Test of semi-continuous version of conflict-based search implemented
// in the satisfiability modulo theories framework.
//
/*----------------------------------------------------------------------------*/


#ifndef __SMTCBS_R_TEST_H__
#define __SMTCBS_R_TEST_H__

#include "core/smtcbsR.h"

using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void);
    
    void test_SMTCBS_R_1(void);
    void test_SMTCBS_R_2(void);
    void test_SMTCBS_R_3(void);
    void test_SMTCBS_R_4(void);            

    
/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __SMTCBS_R_TEST_H__ */
