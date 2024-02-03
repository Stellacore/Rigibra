//
// MIT License
//
// Copyright (c) 2024 Stellacore Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


/*! \file
\brief Unit tests (and example) code for Rigibra::type
*/


#include "_.hpp" // template for header files

#include "type.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace rigibra;

		Transform const gotNullXfm{ null<Transform>() };
		Attitude const gotNullAtt{ null<Attitude>() };
		Location const gotNullLoc{ null<Location>() };

		Transform const gotIdentXfm{ identity<Transform>() };
		Attitude const gotIdentAtt{ identity<Attitude>() };
		Location const gotIdentLoc{ identity<Location>() };

		// [DoxyExample01]

		// check if null instances are not valid

		if ( isValid(gotNullXfm))
		{
			oss << "Failure of gotNullXfm test\n";
		}
		if ( isValid(gotNullAtt))
		{
			oss << "Failure of gotNullAtt test\n";
		}
		if ( isValid(gotNullLoc))
		{
			oss << "Failure of gotNullLoc test\n";
		}

		// check that identity instances are as expected
		using engabra::g3::zero;
		Location const expIdentLoc{ zero<engabra::g3::Vector>() };
		Attitude const expIdentAtt{ zero<engabra::g3::Spinor>() };
		Transform const expIdentXfm{ expIdentLoc, expIdentAtt };

		if (! nearlyEquals(gotIdentLoc, expIdentLoc))
		{
			oss << "Failure of identity Loc test\n";
			oss << "exp: " << expIdentLoc << std::endl;
			oss << "got: " << gotIdentLoc << std::endl;
		}
		if (! nearlyEquals(gotIdentAtt, expIdentAtt))
		{
			oss << "Failure of identity Att test\n";
			oss << "exp: " << expIdentAtt << std::endl;
			oss << "got: " << gotIdentAtt << std::endl;
		}
		if (! nearlyEquals(gotIdentXfm, expIdentXfm))
		{
			oss << "Failure of identity Xfm test\n";
			oss << "exp: " << expIdentXfm << std::endl;
			oss << "got: " << gotIdentXfm << std::endl;
		}

	}

}

//! Check behavior of basic types.
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

