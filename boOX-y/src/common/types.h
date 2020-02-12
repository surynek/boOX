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
/* types.h / 1-224_leibniz                                                    */
/*----------------------------------------------------------------------------*/

#ifndef __TYPES_H__
#define __TYPES_H__


#include <wchar.h>
#include <limits.h>

#include <string>


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

typedef char sInt_8;
typedef unsigned char sUInt_8;

const sInt_8 sINT_8_MAX = CHAR_MAX;
const sInt_8 sINT_8_MIN = CHAR_MIN;

const sUInt_8 sUINT_8_MAX = UCHAR_MAX;
const sUInt_8 sUINT_8_MIN = 0;


/*----------------------------------------------------------------------------*/

typedef short int sInt_16;
typedef unsigned short int sUInt_16;

const sInt_16 sINT_16_MAX = SHRT_MAX;
const sInt_16 sINT_16_MIN = SHRT_MIN;

const sUInt_16 sUINT_16_MAX = USHRT_MAX;
const sUInt_16 sUINT_16_MIN = 0;


/*----------------------------------------------------------------------------*/

typedef int sInt_32;
typedef unsigned int sUInt_32;

const sInt_32 sINT_32_MAX = INT_MAX;
const sInt_32 sINT_32_MIN = INT_MIN;

const sUInt_32 sUINT_32_MAX = UINT_MAX;
const sUInt_32 sUINT_32_MIN = 0;


/*----------------------------------------------------------------------------*/

typedef long long sInt_64;
typedef unsigned long long sUInt_64;

const sInt_64 sINT_64_MAX = LLONG_MAX;
const sInt_64 sINT_64_MIN = LLONG_MIN;

const sUInt_64 sUINT_64_MAX = ULLONG_MAX;
const sUInt_64 sUINT_64_MIN = 0;


/*----------------------------------------------------------------------------*/

typedef char sChar;
typedef wchar_t sWChar;

typedef std::basic_string<sChar> sString;

extern const sString s_INDENT;
extern const sString s2_INDENT;
extern const sString s3_INDENT;
extern const sString s4_INDENT;
extern const sString s5_INDENT;
extern const sString s6_INDENT;
extern const sString s7_INDENT;
extern const sString s8_INDENT;


/*----------------------------------------------------------------------------*/

typedef float sFloat;
typedef double sDouble;

const sDouble s_EPSILON = 0.000000001;
const sDouble s_DELTION = 0.0001;
    

/*----------------------------------------------------------------------------*/

const sUInt_32 sCONVERSION_BUFFER_SIZE = 128;


/*----------------------------------------------------------------------------*/

void sInt_8_to_Screen(sInt_8 int_8);
void sUInt_8_to_Screen(sUInt_8 uint_8);

void sInt_16_to_Screen(sInt_16 int_16);
void sUInt_16_to_Screen(sUInt_16 uint_16);

void sInt_32_to_Screen(sInt_32 int_32);
void sUInt_32_to_Screen(sUInt_32 uint_32);

void sInt_64_to_Screen(sInt_64 int_64);
void sUInt_64_to_Screen(sUInt_64 uint_64);

void sDouble_to_Screen(bool value);
void sBool_to_Screen(bool value);


sString sInt_8_to_String(sInt_8 int_8);
sString sUInt_8_to_String(sUInt_8 uint_8);

sString sInt_16_to_String(sInt_16 int_16);
sString sUInt_16_to_String(sUInt_16 uint_16);

sString sInt_32_to_String(sInt_32 int_32);
sString sUInt_32_to_String(sUInt_32 uint_32);

sString sInt_64_to_String(sInt_64 int_64);
sString sUInt_64_to_String(sUInt_64 uint_64);

sString sDouble_to_String(double value);
sString sBool_to_String(bool value);


/*----------------------------------------------------------------------------*/

sInt_8 sInt_8_from_String(const sString &string);
sUInt_8 sUInt_8_from_String(const sString &string);
sInt_16 sInt_16_from_String(const sString &string);
sUInt_16 sUInt_16_from_String(const sString &string);
sInt_32 sInt_32_from_String(const sString &string);
sUInt_32 sUInt_32_from_String(const sString &string);
sInt_64 sInt_64_from_String(const sString &string);
sUInt_64 sUInt_64_from_String(const sString &string);

double sDouble_from_String(const sString &string);
bool sBool_from_String(const sString &string);


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __TYPES_H__ */

