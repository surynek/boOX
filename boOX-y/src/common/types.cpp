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
/* types.cpp / 1-224_leibniz                                                  */
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>


#include "config.h"
#include "compile.h"
#include "types.h"
#include "defs.h"
#include "result.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    const sString s_INDENT = s__STANDARD_INDENT;
    
    const sString s2_INDENT = s_INDENT + s_INDENT;
    const sString s3_INDENT = s_INDENT + s_INDENT + s_INDENT;
    const sString s4_INDENT = s_INDENT + s_INDENT + s_INDENT + s_INDENT;
    const sString s5_INDENT = s4_INDENT + s_INDENT;
    const sString s6_INDENT = s4_INDENT + s_INDENT + s_INDENT;
    const sString s7_INDENT = s4_INDENT + s_INDENT + s_INDENT + s_INDENT;
    const sString s8_INDENT = s4_INDENT + s_INDENT + s_INDENT + s_INDENT + s_INDENT;

    
/*----------------------------------------------------------------------------*/

    
void sInt_8_to_Screen(sInt_8 int_8)
{
    printf("%d", int_8);
}


void sUInt_8_to_Screen(sUInt_8 uint_8)
{
    printf("%u", uint_8);
}


void sInt_16_to_Screen(sInt_16 int_16)
{
    printf("%d", int_16);
}


void sUInt_16_to_Screen(sUInt_16 uint_16)
{
    printf("%u", uint_16);
}


void sInt_32_to_Screen(sInt_32 int_32)
{
    //    printf("%ld", int_32);
    printf("%d", int_32);
}


void sUInt_32_to_Screen(sUInt_32 uint_32)
{
    //    printf("%lu", uint_32);
    printf("%u", uint_32);
}


void sInt_64_to_Screen(sInt_64 int_64)
{
    printf("%lld", int_64);
}


void sUInt_64_to_Screen(sUInt_64 uint_64)
{
    printf("%llu", uint_64);
}


void sDouble_to_Screen(double value)
{
    printf("%.4f", value);
}


void sBool_to_Screen(bool value)
{
    printf("%s", (value ? "True" : "False"));
}


sString sInt_8_to_String(sInt_8 int_8)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%d", int_8);

    return sString(conversion_buffer);
}


sString sUInt_8_to_String(sUInt_8 uint_8)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%u", uint_8);

    return sString(conversion_buffer);
}


sString sInt_16_to_String(sInt_16 int_16)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%d", int_16);

    return sString(conversion_buffer);
}


sString sUInt_16_to_String(sUInt_16 uint_16)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%u", uint_16);

    return sString(conversion_buffer);
}


sString sInt_32_to_String(sInt_32 int_32)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    //    sprintf(conversion_buffer, "%ld", int_32);
    sprintf(conversion_buffer, "%d", int_32);

    return sString(conversion_buffer);
}


sString sUInt_32_to_String(sUInt_32 uint_32)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    //    sprintf(conversion_buffer, "%lu", uint_32);
    sprintf(conversion_buffer, "%u", uint_32);

    return sString(conversion_buffer);
}


sString sInt_64_to_String(sInt_64 int_64)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%lld", int_64);

    return sString(conversion_buffer);
}


sString sUInt_64_to_String(sUInt_64 uint_64)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%llu", uint_64);

    return sString(conversion_buffer);
}


sString sDouble_to_String(double value)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%.4f", value);

    return sString(conversion_buffer);
}


sString sBool_to_String(bool value)
{
    sChar conversion_buffer[sCONVERSION_BUFFER_SIZE];
    sprintf(conversion_buffer, "%s", value ? "True" : "False");

    return sString(conversion_buffer);
}


/*----------------------------------------------------------------------------*/

sInt_8 sInt_8_from_String(const sString &string)
{
    return atoi(string.c_str());
}


sUInt_8 sUInt_8_from_String(const sString &string)
{
    return atoi(string.c_str());
}


sInt_16 sInt_16_from_String(const sString &string)
{
    return atoi(string.c_str());
}


sUInt_16 sUInt_16_from_String(const sString &string)
{
    return atoi(string.c_str());
}


sInt_32 sInt_32_from_String(const sString &string)
{
    return atol(string.c_str());
}


sUInt_32 sUInt_32_from_String(const sString &string)
{
    return atol(string.c_str());
}


sInt_64 sInt_64_from_String(const sString &string)
{
    return atoll(string.c_str());
}


sUInt_64 sUInt_64_from_String(const sString &string)
{
    return atoll(string.c_str());
}


double sDouble_from_String(const sString &string)
{
    return atof(string.c_str());
}


bool sBool_from_String(const sString &string)
{
    if (string == "True" || string == "true" || string == "TRUE")
    {
	return true;
    }
    else
    {
	if (string == "False" || string == "false" || string == "FALSE")
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	    return false;
	}
    }
}


/*----------------------------------------------------------------------------*/

} // namespace boOX
