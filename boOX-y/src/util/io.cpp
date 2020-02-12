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
/* io.cpp / 1-224_leibniz                                                     */
/*----------------------------------------------------------------------------*/
//
// Input/output functions and utilities.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/times.h>

#include "config.h"
#include "compile.h"

#include "common/types.h"
#include "util/io.h"


using namespace boOX;




/*----------------------------------------------------------------------------*/

namespace boOX
{


    

/*============================================================================*/
// Global functions
/*----------------------------------------------------------------------------*/

    sInt_32 sConsumeUntilChar(FILE *fr, sChar c)
    {
	sInt_32 ch, chars_consumed = 0;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;
	    
	    if ((sChar)ch == c)
	    {
		break;
	    }
	}
	return chars_consumed;
    }

    
    sInt_32 sConsumeUntilString(FILE *fr, const sString &string)
    {
	sInt_32 ch, chars_consumed = 0;
	sString read_string;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;	    

	    if (read_string.length() < string.length())
	    {
		read_string += ch;
	    }
	    else
	    {
		read_string.erase(read_string.begin());
		read_string += ch;
	    }	    
	    if (read_string == string)
	    {
		break;
	    }
	}
	return chars_consumed;
    }


    sInt_32 sConsumeAlphaString(FILE *fr, sString &alpha_string)
    {
	sInt_32 ch, chars_consumed = 0;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;
	    
	    if (!isalpha(ch) && ch != '_')
	    {
		ungetc(ch, fr);
		break;
	    }
	    alpha_string += ch;
	}
	return chars_consumed;
    }


    sInt_32 sConsumeAlnumString(FILE *fr, sString &alnum_string)
    {
	sInt_32 ch, chars_consumed = 0;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;
	    
	    if (!isalnum(ch))
	    {
		ungetc(ch, fr);
		break;
	    }
	    alnum_string += ch;
	}
	return chars_consumed;
    }    


    sInt_32 sConsumeNumericString(FILE *fr, sString &numeric_string)
    {
	sInt_32 ch, chars_consumed = 0;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;
	    
	    if (!isdigit(ch))
	    {
		ungetc(ch, fr);
		break;
	    }
	    numeric_string += ch;
	}
	return chars_consumed;
    }


    sInt_32 sConsumeFloatalString(FILE *fr, sString &floatal_string)
    {
	sInt_32 ch, chars_consumed = 0;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;
	    
	    if (!isdigit(ch) && ch != '.')
	    {
		ungetc(ch, fr);
		break;
	    }
	    floatal_string += ch;
	}
	return chars_consumed;
    }    


    sInt_32 sConsumeWhiteSpaces(FILE *fr)
    {
	sInt_32 ch, chars_consumed = 0;

	while ((ch = fgetc(fr)) != EOF)
	{
	    ++chars_consumed;
	    
	    if (!isspace(ch))
	    {
		ungetc(ch, fr);
		break;
	    }
	}
	return chars_consumed;
    }                

    
/*----------------------------------------------------------------------------*/

} // namespace boOX
