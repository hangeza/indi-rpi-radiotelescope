#ifndef _ASTRO_H
#define _ASTRO_H


#include <complex>
#include <valarray>
#include <vector>
#include <deque>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>
#include <climits>
#include <algorithm>
#include <functional>
#include <numeric>
#include <iterator>
#include <cassert>
#include <cstdarg>
#include <iostream>
#include <ctime>

#include "basic.h"
#include "time.h"

namespace hgz{


/* General Conversions */

template <typename Tp>
Tp DegToRad(const Tp& x)
{
   Tp y;
   y=(Tp)x/360.*twopi;
   return y;
}

template <typename Tp>
Tp RadToDeg(const Tp& x)
{
   Tp y;
   y=x*360./twopi;
   return y;
}

template <typename Tp>
Tp RadToH(const Tp& x)
{
   Tp y;
   y=x*24./twopi;
   return y;
}

template <typename Tp>
Tp HToRad(const Tp& x)
{
   Tp y;
   y=x/24.*twopi;
   return y;
}

template <typename Tp>
Tp DegToH(const Tp& x)
{
   Tp y;
   y=x*24./360.;
   return y;
}

template <typename Tp>
Tp HToDeg(const Tp& x)
{
   Tp y;
   y=x/24.*360.;
   return y;
}


//! general class to hold spherical coordinates
/*!
 * This class consists of a pair of double values which represent angles
 * in radians.
 * It is derived from std::valarray<double> to allow basic math
 * operations such as sin(), cos(), abs(), sqrt() etc.
 */
class SphereCoords : public std::valarray<double>
{
	public:

		/*! initialize with coordinates (0,0)
		 */
		SphereCoords():std::valarray<double>(0.,2){}

		/*! initialize both coordinates with value
			\param init init value
		 */
	   inline SphereCoords(double init):std::valarray<double>(init,2)
		{}

		/*! initialize with given coordinates
			\param phi first coordinate
			\param theta second coordinate
		 */
	   inline SphereCoords(double phi, double theta):std::valarray<double>(0.,2)
		{
			Phi()=phi;
			Theta()=theta;
		}

		/*! copy constructor \n
       *  make deep copy of \e x
		 */
		inline SphereCoords(const SphereCoords& x):std::valarray<double>(0.,2)
		{
			Phi()=x.Phi();
			Theta()=x.Theta();
		}


		/*! standard destructor */
		~SphereCoords() {}
		
		/*! Set Phi-Coordinate */
		double& Phi(){ return (*this)[0]; }
		/*! Set Theta-Coordinate */
		double& Theta(){ return (*this)[1]; }
		
		/*! returns Phi-Coordinate */
		double Phi() const { return (*this)[0]; }
		/*! returns Theta-Coordinate */
		double Theta() const { return (*this)[1]; }

		/*! general Assignment \n
		    (asserts for correct dimensionality)
		 */
		SphereCoords& operator=(const std::valarray<double> &x)
		{
			assert(size()==x.size());
			Phi()=x[0];
			Theta()=x[1];
			return *this;
		}		

		friend SphereCoords operator+(const SphereCoords& x,const SphereCoords& y)
		{
         return SphereCoords(x.Phi()+y.Phi(),x.Theta()+y.Theta());
		}		

		friend SphereCoords operator-(const SphereCoords& x,const SphereCoords& y)
		{
         return SphereCoords(x.Phi()-y.Phi(),x.Theta()-y.Theta());
		}		

		/*! Print information about actual instance to stdout */
		void Print() const
		{
			std::cout<<"\nObject: SphereCoords:"<<std::endl;
			std::cout<<" Address    \t: "<<this<<std::endl;
//			std::cout<<" Components \t: "<<this->size()<<std::endl;
			std::cout<<" Phi        \t: "<<Phi()<<" (rad) ; "
                  <<RadToDeg(Phi())<<" (deg)"<<std::endl;
			std::cout<<" Theta      \t: "<<Theta()<<" (rad) ; "
                  <<RadToDeg(Theta())<<" (deg)"<<std::endl;
		}

		/*! overloaded ostream operator for SphereCoords */
		friend std::ostream& operator<<(std::ostream& o, const SphereCoords &c);

		/*! overloaded istream operator for SphereCoords */
		friend std::istream& operator>>(std::istream& is, SphereCoords &c);

      
		// friend declaration of function distance
      friend double distance(const SphereCoords& x1, const SphereCoords& x2);
};

/*! returns distance of two spherical coordinate-sets \n
	 return-value is the distance in radians of the given points
	 on a Circulum Supremum
 */
double distance(const SphereCoords& x1, const SphereCoords& x2);


/*! overloaded ostream operator for valarray<double> */
std::ostream& operator<<(std::ostream& o, const std::valarray<double> &v);



//! class for handling astronomical nutation
/*!
 * Contains Nutation in longitude, obliquity and ecliptic obliquity.\n
 * Angles are in radians.
 */
class Nutation
{
	private:
      Nutation() {}
   public:
      Nutation(const Time& t);
//      ~Nutation() {}
      
      /*! \return nutation in Longitude in radians */
      double Longitude() const { return _longitude; }
      /*! \return Obliquity in Longitude in radians */
      double Obliquity() const { return _obliquity; }
      /*! \return Obliquity of the Ecliptic in radians */
      double Ecliptic() const { return _ecliptic; }
      
   private:
      static long double c_JD,
                         c_longitude,
                         c_obliquity,
                         c_ecliptic;
	   double _longitude;	/*!< Nutation in longitude */
	   double _obliquity;	/*!< Nutation in obliquity */
	   double _ecliptic;	/*!< Obliquity of the ecliptic */
};




//! Conversion from Horizontal to Equatorial Coordinate system
/*! Transform an objects horizontal into equatorial coordinates
    at given Time \e t and Observer Position \e EarthPos \n
    (Az,Alt) -> (RA,Dec) \n
    \param Hor horizontal Object coordinates
    \param t Time
    \param EarthPos Observer coordinates
    \return calculated equatorial coordinates   
 */    
SphereCoords HorToEqu(const SphereCoords &Hor,
                      const Time& t,
                      const SphereCoords& EarthPos);


//! Conversion from Equatorial to Horizontal Coordinate system
/*! Transform an objects equatorial into horizontal coordinates
    at given Time \e t and Observer Position \e EarthPos \n
    (RA,Dec) -> (Az,Alt) \n
    \param Equ equatorial Object coordinates
    \param t Time
    \param EarthPos Observer coordinates
    \return calculated horizontal coordinates   
 */    
SphereCoords EquToHor(const SphereCoords &Equ,
                      const Time& t,
                      const SphereCoords& EarthPos);
                      


/*!
* \param JD Julian Day
* \return TD
*
* Calculates the dynamical time (TD) difference in seconds (delta T) from 
* universal time.
*/
/* Equation 9.1 on pg 73.
*/

double get_dynamical_time_diff (long double JD);


} // namespace hgz

#endif // _ASTRO_H

