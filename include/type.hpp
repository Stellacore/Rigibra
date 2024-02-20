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

	/*! \brief Physically meaningful 3D angle (size and plane of rotation).
	 *
	 * This class exists to provide type safety and mitigate confusion
	 * working with different angle interpreations. A primary use of
	 * this class is providing type-safe interaction with Attitude
	 * instances.
	 *
	 * Notably this class provides a distinction from SpinAngle.
	 * This PhysAngle type is generally most
	 * useful at the application level and for data i/o.
	 *
	 * Interpretation
	 * \snippet test_type.cpp DoxyExampleAngle
	 *
	 */
	struct PhysAngle
	{
		//! Size and direction of physical rotation angle (default to null).
		engabra::g3::BiVector theBiv
			{engabra::g3::null<engabra::g3::BiVector>()};

	}; // PhysAngle

	/*! \brief Mathematically convenient 3D angle (size and plane of rotation).
	 *
	 * This class exists to provide type safety and mitigate confusion
	 * working with different angle interpreations. A primary use of
	 * this class is providing type-safe interaction with Attitude
	 * instances.
	 *
	 * Notably this class provides a distinction from PhysAngle.
	 * This SpinAngle type is generally most useful for internal
	 * implementations and abstract mathematical relationships.
	 *
	 * The SpinAngle is one half the size of the corresponding PhysAngle.
	 *
	 * Interpretation
	 * \snippet test_type.cpp DoxyExampleAngle
	 */
	struct SpinAngle
	{
		//! 1/2 size and direction of physical rotation angle (default to null).
		engabra::g3::BiVector theBiv
			{engabra::g3::null<engabra::g3::BiVector>()};

		//! SpinAngle extracted from spinor (i.e. logarithm of the spinor).
		inline
		static
		SpinAngle
		from
			( engabra::g3::Spinor const & spin
			)
		{
			return SpinAngle{ engabra::g3::logG2(spin).theBiv };
		}

	}; // PhysAngle


	/*! \brief Attitude of body frame with respect to reference frame.
	 *
	 * Note: The domain of the spinor, and coordiante system in which
	 *       its components are expressed is that of the reference
	 *       frame (with respect to which body attitudes are being
	 *       represented.
	 *
	 * The convention employed is that the attitude refers to that of
	 * the body coordinate frame. I.e., by example:
	 * \snippet test_type.cpp DoxyExampleChirality
	 */
	class Attitude
	{
		/*! \brief Spinor passive convention body wrt reference.
		 *
		 * Convention is:
		 * \arg x: is a vector in the reference frame (the domain)
		 * \arg y: is a vector in the body frame (the transform range)
		 * \arg y = exp(spinAngle) * x * exp(-spinAngle)
		 *
		 * Various optimizations are possible (or desireable for
		 * specific applications) such as caching spinor and or rotation
		 * matrix values (and perhaps also their inverses). However,
		 * For present purpose, it's most convenient to store the angle
		 * itself to provide a simple and clean representation. Also,
		 * this allows returning the physical angle withouth having
		 * to resolve ambiquities associated with cases in which the
		 * plane of rotation is fundamentally undefined.
		 */
		SpinAngle theSpinAngle{ engabra::g3::null<engabra::g3::BiVector>() };

	public:

		//! Construct a null instance
		inline
		explicit
		Attitude
			()
			: theSpinAngle{ engabra::g3::null<engabra::g3::BiVector>() }
		{ }

		//! Construct attitude of body matching this physical angle.
		inline
		explicit
		Attitude
			( PhysAngle const & physAngle
			)
			: theSpinAngle{ .5 * physAngle.theBiv }
		{ }

		//! Construct attitude of body matching TWICE this.
		inline
		explicit
		Attitude
			( SpinAngle const & spinAngle
			)
			: theSpinAngle{ spinAngle }
		{ }

		//! Construct with SpinAngle extracted directly from this spinor.
		inline
		explicit
		Attitude
			( engabra::g3::Spinor const & spin
			)
			: theSpinAngle{ SpinAngle::from(spin) }
		{ }

		/*! Spinor representation of attitude
		 *
		 * Convention is:
		 * \arg x: is a vector in the reference frame (the domain)
		 * \arg y: is a vector in the body frame (the transform range)
		 * \arg y = spinor() * x * reverse(spinor())
		 */
		inline
		engabra::g3::Spinor
		spinor
			() const
		{
			return engabra::g3::exp(theSpinAngle.theBiv);
		}

		//! Spinor angle associated with the Attitude (half of physAngle());
		inline
		SpinAngle const &
		spinAngle
			() const
		{
			return theSpinAngle;
		}

		//! Physical angle associated with the Attitude (twice spinAngle()).
		inline
		PhysAngle
		physAngle
			() const
		{
			return PhysAngle{ 2. * theSpinAngle.theBiv };
		}

		//! Expressed of vector in range(into) frame equiv to vecFrom in domain.
		inline
		engabra::g3::Vector
		operator()
			( engabra::g3::Vector const & vecFrom
			) const
		{
			using namespace engabra::g3;
			// Note: this can be optimized - e.g. Create a FastAttitude
			// class that constructs spinors (and/or grid matrices) for
			// subsequent use in transforming large quantities of data
			Spinor const spin{ spinor() };
			return (spin * vecFrom * reverse(spin)).theVec;
		}

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
	 *
	 * Example:
	 * \snippet test_type.cpp DoxyExampleOrderTR
	 */
	struct Transform
	{
		/*! \brief Location of body expressed in reference system.
		 */
		engabra::g3::Vector theLoc{ engabra::g3::null<engabra::g3::Vector>() };

		/*! \brief Attitude of body with respect to reference frame.
		 */
		Attitude theAtt
			{ PhysAngle{ engabra::g3::null<engabra::g3::BiVector>() } };

		//! Expressed of vector in range(into) frame equiv to vecFrom in domain.
		inline
		engabra::g3::Vector
		operator()
			( engabra::g3::Vector const & vecFrom
			) const
		{
			using namespace engabra::g3;
			return theAtt(vecFrom - theLoc);
		}

	}; // Transform


//
// == constant types
//

	/*! Component instances of object that support identity transformation.
	 *
	 * E.g. for
	 * \arg PhysAngle
	 * \arg SpinAngle
	 */
	template <typename Type>
	inline
	Type
	identity
		()
	{
		// as implemented here, zero parameter values provide identity
		return engabra::g3::zero<Type>();
	}

	//! Specialization for Attitude
	template <>
	inline
	Attitude
	identity
		()
	{
		// as implemented here, zero parameter values provide identity
		using namespace engabra::g3;
		return Attitude{ SpinAngle{ zero<BiVector>() } };
	}

	//! Specialization for Transform
	template <>
	inline
	Transform
	identity
		()
	{
		// as implemented here, zero parameter values provide identity
		using namespace engabra::g3;
		return Transform{ zero<Vector>(), identity<Attitude>() };
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
	null
		()
	{
		using namespace engabra::g3;
		return Attitude(PhysAngle{ null<BiVector>() });
	}


	//! Provide explicit implementation for null<Transform>
	template <>
	inline
	Transform
	null
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
			( engabra::g3::isValid(att.spinAngle().theBiv)
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
		return engabra::g3::nearlyEquals
			(attA.spinAngle().theBiv, attB.spinAngle().theBiv, tol);
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
	//! Overload for putting PhysAngle to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, rigibra::PhysAngle const & physAngle
		)
	{
		using namespace engabra::g3;
		ostrm << physAngle.theBiv;
		return ostrm;
	}

	//! Overload for putting SpinAngle to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, rigibra::SpinAngle const & spinAngle
		)
	{
		using namespace engabra::g3;
		ostrm << spinAngle.theBiv;
		return ostrm;
	}


	//! Overload for putting attitude spinor contents to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, rigibra::Attitude const & att
		)
	{
		ostrm << "physAngle: " << att.physAngle().theBiv;
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
		ostrm
			<< " loc: " << xfm.theLoc
			<< " att: " << xfm.theAtt
			;
		return ostrm;
	}

} // [anon]

#endif // Rigibra_type_INCL_
