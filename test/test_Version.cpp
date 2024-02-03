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
\brief Unit tests (and example) code for Rigibra version information.
*/


#include "Rigibra.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace
{
	//! Example for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		std::string const gotVer{ rigibra::projectVersion() };
		std::string const gotSid{ rigibra::sourceIdentity() };

		// [DoxyExample01]

		constexpr std::string::value_type aDot('.');
		constexpr long int expDotCount{ 2u };
		long int const gotDotCount
			{ std::count(gotVer.cbegin(), gotVer.cend(), aDot) };
		if (! (gotDotCount == expDotCount))
		{
			oss << "Failure of projectVersion string test\n";
			oss << "expDotCount: " << expDotCount << '\n';
			oss << "gotDotCount: " << gotDotCount << '\n';
			oss << "     gotVer: '" << gotVer << "'\n";
		}

		std::string const tagStart("t_");
		bool const gotTagStart{ std::string::npos != gotSid.find(tagStart) };
		if (! gotTagStart)
		{
			oss << "Failure of sourceIdentity string test\n";
			oss << " got: '" << gotSid << "'\n";
			oss << "    : perhaps need git config to enable 'git describe'\n";
			oss << "    : with command similar to:\n";
			oss << "    : 'git config "
				"--global --add safe.directory /repos/Rigibra'" << '\n';
		}
	}
}

//! Check behavior of NS
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	std::cout << "=== Rigibra: Project Version: "
		<< rigibra::projectVersion() << '\n';
	std::cout << "=== Rigibra: Source Identity: "
		<< rigibra::sourceIdentity() << '\n';
	std::cout << "=== Using Engabra Project Version: "
		<< engabra::projectVersion() << '\n';
	std::cout << "=== Using Engabra Source Identity: "
		<< engabra::sourceIdentity() << '\n';

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

