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


#ifndef Rigibra_type_INCL_
#define Rigibra_type_INCL_

/*! \file
\brief Contains fundamental types for expressing rigid body orientations.

Example:
\snippet test_type.cpp DoxyExample01

*/

#include <Engabra>

#include <limits>


namespace rigibra
{

	using Location = engabra::g3::Vector;

	/*! \brief Attitude of body frame with respect to reference frame.
	 *
	 * Note: The domain of the spinor, and coordiante system in which
	 *       its components are expressed is that of the reference
	 *       frame (with respect to which body attitudes are being
	 *       represented.
	 */
	struct Attitude
	{
		/*! \brief Spinor passive convention body wrt reference.
		 *
		 * Convention is:
		 * \arg x: is a vector in the reference frame (the domain)
		 * \arg y: is a vector in the body frame (the transform range)
		 * \arg y = theSpin * x * reverse(theSpin)
		 */
		engabra::g3::Spinor theSpin{};

	}; // Attitude

	/*! \brief Rigid Body Location and Attitude represention in 3D.
	 *
	 * Conventions include:
	 * \arg Transformation represents the offset/attitude of the
	 *      coordiante systems (passive transformation convention).
	 * \arg Transformation parameters are all expressed in the
	 *      transformation domain (the reference coordinate system).
	 * \arg Interpreation is that of actions performed on the body
	 *      coordiante system in the order "translation and then rotate".
	 *
	 * Math convention is:
	 * \arg x: is the expression of a vector as it is represented
	 *         in the reference frame (the transformation domain).
	 * \arg t: is the translation vector (expressed in the reference
	 *         frame) and represents the amount by which the body
	 *         is translated (before being rotated).
	 * \arg y: is the expression of the same vector as it is represented
	 *         in the body frame (the transform range).
	 * \arg y = theSpin * (x - t) * reverse(theSpin)
	 *
	 * Note: All internal data members have values expressed in the
	 *       transformation domain (the reference coordiante system).
	 *       I.e., the components of #theAtt are expressed in the
	 *       same coordinate frame as are the components of #theLoc.
	 */
	struct Transform
	{
		/*! \brief Location of body expressed in reference system.
		 */
		engabra::g3::Vector theLoc{};

		/*! \brief Attitude of body with respect to reference frame.
		 *
		 */
		Attitude theAtt{};

	}; // Transform


//
// == constant types
//

	//! Component instances of object that support identity transformation.
	template <typename Type>
	inline
	Type
	identity
		()
	{
		// as implemented here, zero parameter values provide identity
		return engabra::g3::zero<Type>();
	}

	//! In general forward null object requests to Engabra
	template <typename Type>
	inline
	Type
	constexpr null
		()
	{
		return engabra::g3::null<Type>();
	}

	//! Provide explicit implementation for null<Attitude>
	template <>
	inline
	Attitude
	constexpr null
		()
	{
		return Attitude{ engabra::g3::null<engabra::g3::Spinor>() };
	}


	//! Provide explicit implementation for null<Transform>
	template <>
	inline
	Transform
	constexpr null
		()
	{
		return Transform{ null<Location>(), null<Attitude>() };
	}

//
// Validity
//

	//! Provide explicit implementation for isValid<Attitude>
	inline
	bool
	isValid
		( Attitude const & att
		)
	{
		return
			( engabra::g3::isValid(att.theSpin)
			);
	}

	//! Provide explicit implementation for isValid<Transform>
	inline
	bool
	isValid
		( Transform const & xform
		)
	{
		return
			(  engabra::g3::isValid(xform.theLoc)
			&& isValid(xform.theAtt)
			);
	}

//
// Comparison
//

	//! True if member data values are same within tolerance
	inline
	bool
	nearlyEquals
		( Attitude const & attA
		, Attitude const & attB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return engabra::g3::nearlyEquals(attA.theSpin, attB.theSpin, tol);
	}

	//! True if member data values are same within tolerance
	inline
	bool
	nearlyEquals
		( Transform const & xfmA
		, Transform const & xfmB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return
			(  engabra::g3::nearlyEquals(xfmA.theLoc, xfmB.theLoc, tol)
			&& nearlyEquals(xfmA.theAtt, xfmB.theAtt, tol)
			);
	}



} // [rigibra]

//
// I/O
//

namespace
{
	//! Overload for putting attitude spinor contents to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, rigibra::Attitude const & att
		)
	{
		ostrm << att.theSpin;
		return ostrm;
	}

	//! Overload for putting attitude spinor contents to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, rigibra::Transform const & xfm
		)
	{
		ostrm << xfm.theLoc << "  " << xfm.theAtt;
		return ostrm;
	}

} // [anon]

#endif // Rigibra_type_INCL_
