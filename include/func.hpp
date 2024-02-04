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


#ifndef Rigibra_func_CN_INCL_
#define Rigibra_func_CN_INCL_

/*! \file
\brief Contains functions for operating with rigid body objects.

Example:
\snippet test_func.cpp DoxyExample01

*/


#include "type.hpp"


namespace rigibra
{

	//! Inverse Attitude (such that return*fwd = identity)
	inline
	Attitude
	inverse
		( Attitude const & fwd
		)
	{
		using namespace engabra::g3;
		return Attitude{ Spinor{ fwd.theSpin.theSca, -fwd.theSpin.theBiv } };
	}

	//! Inverse Transformation (such that return*fwd = identity)
	inline
	Transform
	inverse
		( Transform const & fwd
		)
	{
		Attitude const invAtt{ inverse(fwd.theAtt) };
		Location const invLoc{ -fwd.theAtt(fwd.theLoc) };
		return Transform{ invLoc, invAtt };
	}

	//! Composition of two Attitudes: returns attBwX = attBwA * attAwX.
	inline
	Attitude
	operator*
		( Attitude const & attBwA
		, Attitude const & attAwX
		)
	{
	//	Attitude const & attA = attAwX;
	//	Attitude const & attB = attBwA;
	//	return Attitude{ attB * attA };
		return Attitude{ attBwA.theSpin * attAwX.theSpin };
	}

	/*! Composition Transformations result is xBwRef(pnt) = xBwA(xAwRef(pnt)).
	 *
	 * Ref theory/Transforms.lyx document for math details. Overall, the
	 * composition is defined such that resulting transform, xBwRef has
	 * location constitutent, locBinRef, and attitude constituent, attBwRef.
	 *
	 * I.e. result, xBwRef, is of the form:
	 * \arg xBwRef(pnt) = attBwRef( pnt - locBinRef )
	 *
	 * and result, xBwRef, has the property that:
	 * \arg xBwRef(pnt) = xBwA( xAwRef(pnt) )
	 */
	inline
	Transform
	operator*
		( Transform const & xBwA //!< e.g. xfm frameB wrt frameA
		, Transform const & xAwRef //!< e.g. xfm frameA wrt frameRef
		)
	{
		Attitude const & attA = xAwRef.theAtt;
		Attitude const & attB = xBwA.theAtt;
		Attitude const attBwRef{ attB * attA };
		Attitude const invA{ inverse(xAwRef.theAtt) };
		Location const & aLoc = xAwRef.theLoc;
		Location const & bLoc = xBwA.theLoc;
		Location const locBinRef{ aLoc + invA(bLoc) };
		return Transform{ locBinRef, attBwRef };
	}

} // [rigibra]


#endif // Rigibra_func_CN_INCL_
