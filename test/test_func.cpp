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
\brief Unit tests (and example) code for rigibra::func
*/


#include "func.hpp"

#include <iostream>
#include <limits>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	testIdentInverses
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace rigibra;

		// identity attitude is its own inverse
		Attitude const attIdent2w1{ identity<Attitude>() };
		Attitude const attIdent1w2{ inverse(attIdent2w1) };

		// identity transforms is its own inverse
		Transform const xfmIdent2w1{ identity<Transform>() };
		Transform const xfmIdent1w2{ inverse(xfmIdent2w1) };

		// identity transformation produces no change in vector
		Transform const ident{ identity<Transform>() };
		using engabra::g3::Vector;
		Vector const aVec{ -4., 3., 7. };
		Vector const bVec{ ident(aVec) }; // same as aVec

		// [DoxyExample01]

		if (! nearlyEquals(attIdent2w1, attIdent1w2))
		{
			oss << "Failure of attIdent test\n";
			oss << "attIdent2w1: " << attIdent2w1 << '\n';
			oss << "attIdent1w2: " << attIdent1w2 << '\n';
		}

		if (! nearlyEquals(xfmIdent2w1, xfmIdent1w2))
		{
			oss << "Failure of xfmIdent test\n";
			oss << "xfmIdent2w1: " << xfmIdent2w1 << '\n';
			oss << "xfmIdent1w2: " << xfmIdent1w2 << '\n';
		}

		if (! nearlyEquals(bVec, aVec))
		{
			oss << "Failure of identity {a,b}Vec transform test\n";
			oss << "aVec: " << aVec << '\n';
			oss << "bVec: " << bVec << '\n';
		}

	}

	//! Check transformation inversion
	void
	testInverse
		( std::ostream & oss
		)
	{
		// construct arbitrary rigid body transform
		using namespace rigibra;
		PhysAngle const physAngle{ -2., 1., .75 };
		Attitude const att{ Attitude(physAngle) };
		Location const loc{ -5., 2., -3. };
		Transform const xfm{ (1./8.)*loc, att };

		using namespace engabra::g3;
		Vector const expVecX{ -11., 17., -19. };
		Vector const gotVecY{ xfm(expVecX) };
		Vector const gotVecX{ inverse(xfm)(gotVecY) };

		if (! nearlyEquals(gotVecX, expVecX))
		{
			Vector const difVecX{ gotVecX - expVecX };
			oss << "Failure of Transform inverse test\n";
			oss << "expVecX: " << expVecX << std::endl;
			oss << "gotVecX: " << gotVecX << std::endl;
			oss << "difVecX: " << io::enote(difVecX, 5u) << std::endl;
		}
	}

	//! Check composition of two transformations
	void
	testComposite
		( std::ostream & oss
		)
	{
		using namespace rigibra;
		using namespace engabra::g3;

		Location const locA{ 10., 20., 30. };
		PhysAngle const angA{ turnQtr * e12 };
		Transform const xfmAwR{ locA, Attitude(angA) };

		Vector const aPnt{ 10., 21., 30. };
		Vector const expPntA{ 1., 0., 0. };
		Vector const gotPntA{ xfmAwR(aPnt) };

		// adjust test tolerance to be compatible with translation magnitudes
		static double tol
			{ magnitude(locA) * std::numeric_limits<double>::epsilon() };
		if (! nearlyEquals(gotPntA, expPntA, tol))
		{
			oss << "Failure of composite pntA test\n";
			oss << "exp: " << expPntA << '\n';
			oss << "got: " << gotPntA << '\n';
		}

		Location const locB{ 1., 2., 3. };
		PhysAngle const angB{ turnQtr * e23 };
		Transform const xfmBwA{ locB, Attitude(angB) };

		Vector const expPntB{ 0., -3., 2. };
		Vector const gotPntB{ xfmBwA(gotPntA) };

		if (! nearlyEquals(gotPntB, expPntB, tol))
		{
			oss << "Failure of composite pntB test\n";
			oss << "exp: " << expPntB << '\n';
			oss << "got: " << gotPntB << '\n';
		}

		Transform const xfmBwR{ xfmBwA * xfmAwR };
		Vector const expPntC{ expPntB };
		Vector const gotPntC{ xfmBwR(aPnt) };
		Vector const chkPntC{ xfmBwA(xfmAwR(aPnt)) };

		if (! nearlyEquals(gotPntC, expPntC, tol))
		{
			oss << "Failure of composite pntC test\n";
			oss << "exp: " << expPntC << '\n';
			oss << "got: " << gotPntC << '\n';
			oss << "chk: " << chkPntC << '\n';
		}

		// check arbitrary direction

		// [DoxyExampleComposite]

		// Example of an item mounted arbitrarily onto some body
		Location const locItemOnBody{ -3., 2., .5 };
		PhysAngle const angleItemWrtBody
			{ (pi/7.) * direction(17.*e23 - 13.*e31 + 11.*e12) };
		Attitude const attItemWrtBody{ Attitude(angleItemWrtBody) };
		Transform const xfmItemWrtBody{ locItemOnBody, attItemWrtBody };

		// Example of a body oriented in some reference frame
		Location const locBodyInRef{ 30., -10., -20. };
		PhysAngle const angleBodyWrtRef{ BiVector{ .75, .25, -.5 } };
		Attitude const attBodyWrtRef{ Attitude(angleBodyWrtRef) };
		Transform const xfmBodyWrtRef{ locBodyInRef, attBodyWrtRef };

		// Use composition to determine location of item in reference frame
		Transform const xfmItemWrtRef{ xfmItemWrtBody * xfmBodyWrtRef };

		// Composition provided same result as applying each tranform in order
		Vector const expPntInRef{ .5, .2, -.3 };
		// one step
		Vector const gotPntInItem1{ xfmItemWrtRef(expPntInRef) };
		// two steps
		Vector const gotPntInBody2{ xfmBodyWrtRef(expPntInRef) };
		Vector const gotPntInItem2{ xfmItemWrtBody(gotPntInBody2) };

		if (! nearlyEquals(gotPntInItem2, gotPntInItem1, tol))
		{
			oss << "Failure of general composition one vs two step test\n";
			oss << "gotPntInItem1: " << gotPntInItem1 << '\n';
			oss << "gotPntInItem2: " << gotPntInItem2 << '\n';
		}

		// Utilize inverse transformations to transform in other direction

		Transform const xfmBodyWrtItem{ inverse(xfmItemWrtBody) };
		Vector const invPntInBody2{ xfmBodyWrtItem(gotPntInItem2) };

		if (! nearlyEquals(invPntInBody2, gotPntInBody2, tol))
		{
			oss << "Failure of general composition inverse InBody test\n";
			oss << "gotPntInBody2: " << gotPntInBody2 << '\n';
			oss << "invPntInBody2: " << invPntInBody2 << '\n';
		}

		Transform const xfmRefWrtBody{ inverse(xfmBodyWrtRef) };
		Vector const invPntInRef2{ xfmRefWrtBody(invPntInBody2) };

		if (! nearlyEquals(invPntInRef2, expPntInRef, tol))
		{
			oss << "Failure of general composition inverse InRef test\n";
			oss << " expPntInRef: " << expPntInRef << '\n';
			oss << "invPntInRef2: " << invPntInRef2 << '\n';
		}

		Transform const xfmRefWrtItem{ xfmRefWrtBody * xfmBodyWrtItem };
		Vector const invPntInRef3{ xfmRefWrtItem(gotPntInItem2) };

		if (! nearlyEquals(invPntInRef3, expPntInRef, tol))
		{
			oss << "Failure of general composition inverse one step test\n";
			oss << "invPntInRef3: " << invPntInRef3 << '\n';
			oss << " expPntInRef: " << expPntInRef << '\n';
		}

	}
}

//! Check behavior of func
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testIdentInverses(oss);
	testInverse(oss);
	testComposite(oss);

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

