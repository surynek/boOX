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
/* graph_test.h / 0_iskra-161                                                 */
/*----------------------------------------------------------------------------*/
//
// Graph data structures and algorithms - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __GRAPH_TEST_H__
#define __GRAPH_TEST_H__

#include "core/graph.h"

using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    void print_Introduction(void);
    
    void test_undirected_graph_1(void);
    void test_undirected_graph_2(int N_Vertices, double edge_prob);
    void test_undirected_graph_3(const sString &filename);
    void test_undirected_graph_4(void);

    void test_statistics_1(void);

    
/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __GRAPH_TEST_H__ */
