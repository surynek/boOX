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
/* defs.h / 1-224_leibniz                                                     */
/*----------------------------------------------------------------------------*/

#ifndef __DEFS_H__
#define __DEFS_H__

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "compile.h"


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// Auxiliary macros

#define sUNUSED(x)

#ifdef sDEBUG
  #define sUNUSED_(x) x
#else
  #define sUNUSED_(x)
#endif

#ifdef sDEEP_CONSISTENCY
  #define sUSED_(x) x
#else
  #define sUSED_(x)
#endif


/*----------------------------------------------------------------------------*/
// Maximum, minimum macros

#define sMIN(x,y) (((x) < (y)) ? (x) : (y))
#define sMAX(x,y) (((x) > (y)) ? (x) : (y))
#define sDFR(x,y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))
#define sABS(x) (((x) < 0) ? -(x) : (x))
#define sSGN(x) (((x) < 0) ? -(-1) : ((x) > 0) ? 1 : 0)


/*----------------------------------------------------------------------------*/
// Auxiliary macros

#define sSWAP(x, y, T)      \
    {                       \
        T z;                \
	z = x;              \
	x = y;              \
	y = z;              \
    }


/*----------------------------------------------------------------------------*/
// C++ Language extensions

#define SEMICONST


/*----------------------------------------------------------------------------*/
// Size definitions

#define sKILO(x) (1024 * (x))
#define sMEGA(x) (1024 * 1024 * (x))
#define sGIGA(x) (1024 * 1024 * 1024 * (x))


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __DEFS_H__ */
