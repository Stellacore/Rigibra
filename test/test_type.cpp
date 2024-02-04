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


#include "type.hpp"
#include "func.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	testNullIdent
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
		Location const expIdentLoc{ engabra::g3::Vector{ 0., 0., 0. } };
		Attitude const expIdentAtt{ engabra::g3::Spinor{ 1., 0., 0., 0. } };
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

	//! Construct attitude instances in various ways
	void
	testAttCtor
		( std::ostream & oss
		)
	{
		// [DoxyExampleAngle]

		// used for basic types: e.g., pi, e12, exp(), Vector, Spinor, etc.
		using namespace engabra::g3;

		// This represents the physical rotation in which a
		// body turns 1/6 of a turn (60 deg) with the rotation
		// in the e1^e2 (aka X-Y) plane.
		rigibra::PhysAngle const expPhysAngle{ (pi/3.) * e12 };

		// The spin angle is one half of the physical angle...
		rigibra::SpinAngle const expSpinAngle{ .5 * (pi/3.) * e12 };

		// The 1/2 relationship is because, when rotation is performed
		// by spinor multiplication, the spinor appears twice in the
		// canonical rotation formula. This has the effect of
		// doubling the SpinAngle. For example,
		Spinor const spin{ exp(expSpinAngle.theBiv) };
		Vector const expVecX{ .2, -.5, -.3 };
		Vector const gotVecY{ (spin * expVecX * reverse(spin)).theVec };

		// The two distinct angle types can be used to create
		// Attitude instances that are consistent with each other. I.e.,
		// Each represents the exact same rotation and attitude.
		rigibra::Attitude const gotAttFromPhys
			{ rigibra::Attitude::from(expPhysAngle) };
		rigibra::Attitude const gotAttFromSpin
			{ rigibra::Attitude::from(expSpinAngle) };

		// Since the attitudes are the same, so are the inverses
		rigibra::Attitude const attPhysYwX{ inverse(gotAttFromPhys) };
		rigibra::Attitude const attSpinYwX{ inverse(gotAttFromSpin) };

		// And vectors transform the same using any Attitude instance
		// no matter what angle type is used to construct it.
		Vector const gotPhysVecX{ attPhysYwX(gotVecY) };
		Vector const gotSpinVecX{ attSpinYwX(gotVecY) };

		// [DoxyExampleAngle]

		if (! nearlyEquals(gotPhysVecX, gotSpinVecX))
		{
			oss << "Failure of got{Phys,Spin}VecX test\n";
			oss << "gotPhysVecX: " << gotPhysVecX << '\n';
			oss << "gotSpinVecX: " << gotSpinVecX << '\n';
		}
	}

	//! Check conventions (e.g. passive rotation, translate then rotate)
	void
	testConvention
		( std::ostream & oss
		)
	{
		// [DoxyExampleChirality]

		using namespace rigibra;
		using namespace engabra::g3;

		// Small turn of body about the X-Y plane
		PhysAngle const smallTurnCCW{ .1 * e12 };
		Attitude const att{ Attitude::from(smallTurnCCW) };

		// Since the body is turned counter clock wise (CCW),
		// the world frame "x-axis" should appear to have a
		// negative "y-component" when expressed in the body frame.
		Vector const expNegY{ att(e1) };

		// check convention
		if (! (expNegY[1] < 0.))
		{
			oss << "Failure of rotation chirality test\n";
			oss << "expNegY: " << expNegY << '\n';
		}

		// [DoxyExampleChirality]

		// [DoxyExampleOrderTR]

		using namespace rigibra;
		using namespace engabra::g3;

		// Center of body frame should be (and stay) at this location
		Location const expBodyOrigInRef{ e1 };

		// Body attitude set to a 90-deg rotation in the "x-y" plane
		PhysAngle const angleQtr{ turnQtr * e12 };
		Attitude const attQtr{ Attitude::from(angleQtr) };

		// Put body at expBodyOrigInRef and THEN rotated 90-deg CCW in X-Y plane
		Transform const xfmBodyWrtRef{ expBodyOrigInRef, attQtr };

		// Body frame origin should be at expBodyOrigInRef
		Vector const bodyOrigInBody{ zero<Vector>() };
		Transform const xfmRefWrtBody{ inverse(xfmBodyWrtRef) };
		Vector const gotBodyOrigInRef{ xfmRefWrtBody(bodyOrigInBody) };

		// check convention
		if (! nearlyEquals(gotBodyOrigInRef, expBodyOrigInRef))
		{
			// note basic Engabra operations and i/o support
			Vector const difBodyOrigInRef
				{ gotBodyOrigInRef - expBodyOrigInRef };
			oss << "Failure of translate/rotate order test\n";
			oss << "expBodyOrigInRef: " << expBodyOrigInRef << '\n';
			oss << "gotBodyOrigInRef: " << gotBodyOrigInRef << '\n';
			oss << "difBodyOrigInRef: " << io::enote(difBodyOrigInRef) << '\n';
			oss << "xfmBodyWrtRef: " << xfmBodyWrtRef << '\n';
			oss << "xfmRefWrtBody: " << xfmRefWrtBody << '\n';
		}

		// [DoxyExampleOrderTR]
	}

}

//! Check behavior of basic types.
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	testNullIdent(oss);
	testAttCtor(oss);
	testConvention(oss);

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

