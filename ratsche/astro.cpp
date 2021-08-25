#include <stdio.h>
#include <typeinfo>
#include <climits>
#include <cassert>
#include <iostream>
#include <ios>
#include <ctime>

#include "basic.h"
#include "time.h"
#include "astro.h"


using namespace std;

namespace hgz{



SphereCoords HorToEqu(const SphereCoords &Hor,
                      const Time& t,
                      const SphereCoords& EarthPos)
{
   SphereCoords Equ;
   
	long double H, longitude, declination, latitude, A, h, sidereal, JD;

	/* object alt/az */
	A = Hor.Phi();
	h = Hor.Theta();

	/* observer long / lat */
	longitude = EarthPos.Phi();
	latitude = EarthPos.Theta();

	/* equ on pg89 */
	H = atan2 (sin (A), ( cos(A) * sin (latitude) + tan(h) * cos (latitude)));
	
   declination = sin(latitude) * sin(h) - cos(latitude) * cos(h) * cos(A);
	declination = asin (declination);

	/* get ra = sidereal - longitude + H */
//   JD=t.JD();
//	sidereal = ln_get_mean_sidereal_time(JD);
//	sidereal = t.MeanSidereal();
	sidereal = t.ApparentSidereal();
   sidereal *= twopi/24.;

	Equ.Phi() = modpi2 (sidereal - H + longitude);
	Equ.Theta() = modpi2 (declination);

   return Equ;
}


SphereCoords EquToHor(const SphereCoords &Equ,
                      const Time& t,
                      const SphereCoords& EarthPos)
{
   SphereCoords Hor;

	long double H, ra, latitude, declination, sidereal, A, Ac, As, h, Z, Zs;

	/* change sidereal_time from hours to radians*/
//	sidereal = get_mean_sidereal_time(t.JD());
//	sidereal = t.MeanSidereal();
	sidereal = t.ApparentSidereal();
   sidereal *= twopi / 24.0;

	/* calculate hour angle of object at observers position */
	ra = Equ.Phi();
	H = sidereal + EarthPos.Phi() - ra;

	/* hence formula 12.5 and 12.6 give */
	/* convert to radians - hour angle, observers latitude, object declination */
	latitude = EarthPos.Theta();
	declination = Equ.Theta();

	/* formula 12.6 *; missuse of A (you have been warned) */
	A = sin (latitude) * sin (declination) + cos (latitude) * cos (declination) * cos (H);
	h = asin (A);

	/* convert back to degrees */
	Hor.Theta() = h;   

	/* zenith distance, Telescope Control 6.8a */
	Z = acos (A);

	/* is'n there better way to compute that? */
	Zs = sin (Z);

	/* sane check for zenith distance; don't try to divide by 0 */

	if (Zs < 1e-5) {
		if (modpi(latitude) > 0)
			Hor.Phi() = 180;
		else
			Hor.Phi() = 0;
		return Hor;
	}

	/* formulas TC 6.8d Taff 1991, pp. 2 and 13 - vector transformations */
	As = (cos (declination) * sin (H)) / Zs;
	Ac = (sin (latitude) * cos (declination) * cos (H) - cos (latitude) * sin (declination)) / Zs;
   
   // don't blom at atan2
	if (fabs(As) < 1e-5) {
		Hor.Phi() = 0;
		return Hor;
	}
	A = atan2 (As, Ac);

	// normalize A
	A = modpi2(A);

	/* convert back to degrees */
	Hor.Phi()=A;
   
   return Hor;
}





ostream& operator<<(ostream& o, const SphereCoords &c)
{
	o<<"("<<c[0]<<","<<c[1]<<")";
	return o;
}

istream& operator>>(istream& is, SphereCoords &c)
{
   complex_t comp;
   char ch;
   is>>ch;
   if (ch=='(')
   {
      is>>c.Phi()>>ch;
      if (ch==',')
      {
         is>>c.Theta()>>ch;
         if (ch!=')') is.setstate(ios_base::failbit);
      }
      else is.setstate(ios_base::failbit);
      
   }
   else
   {
      is.putback(ch);
      is>>c.Phi()>>c.Theta();
   }
   
//   c.Print();
   return is;
   
   is>>comp;
   c.Phi()=real(comp);
   c.Theta()=imag(comp);
   return is;
}

ostream& operator<<(ostream& o, const valarray<double> &v)
{
	o<<"(";
   for (size_t i=0;i<v.size();++i)
   {
      o<<v[i]<<",";
	}
//   std::copy(v[0],v[v.size()],std::ostream_iterator<double> >(o,","));
	o<<"\b)";
	return o;
}


double distance(const SphereCoords& x1, const SphereCoords& x2)
{
   SphereCoords deltax;
	deltax=x1-x2;
//   deltax.Print();
	return acos(cos(deltax.Phi())*cos(deltax.Theta()));
}



/*
 *
 *
 * Definitions for class Nutation
 *
 *
 *
 */

#define NUTATION_TERMS 63
#define NUTATION_EPOCH_THRESHOLD 0.1

struct nutation_arguments
{
    double D;
    double M;
    double MM;
    double F;
    double O;
};

struct nutation_coefficients
{
    double longitude1;
    double longitude2;
    double obliquity1;
    double obliquity2;
};

/* arguments and coefficients taken from table 21A on page 133 */

static const struct nutation_arguments arguments[NUTATION_TERMS] = {
    {0.0,	0.0,	0.0,	0.0,	1.0},
    {-2.0,	0.0,	0.0,	2.0,	2.0},
    {0.0,	0.0,	0.0,	2.0,	2.0},
    {0.0,	0.0,	0.0,	0.0,	2.0},
    {0.0,	1.0,	0.0,	0.0,	0.0},
    {0.0,	0.0,	1.0,	0.0,	0.0},
    {-2.0,	1.0,	0.0,	2.0,	2.0},
    {0.0,	0.0,	0.0,	2.0,	1.0},
    {0.0,	0.0,	1.0,	2.0,	2.0},
    {-2.0,	-1.0,	0.0,	2.0,	2.0},
    {-2.0,	0.0,	1.0,	0.0,	0.0},
    {-2.0,	0.0,	0.0,	2.0,	1.0},
    {0.0,	0.0,	-1.0,	2.0,	2.0},
    {2.0,	0.0,	0.0,	0.0,	0.0},
    {0.0,	0.0,	1.0,	0.0,	1.0},
    {2.0,	0.0,	-1.0,	2.0,	2.0},
    {0.0,	0.0,	-1.0,	0.0,	1.0},
    {0.0,	0.0,	1.0,	2.0,	1.0},
    {-2.0,	0.0,	2.0,	0.0,	0.0},
    {0.0,	0.0,	-2.0,	2.0,	1.0},
    {2.0,	0.0,	0.0,	2.0,	2.0},
    {0.0,	0.0,	2.0,	2.0,	2.0},
    {0.0,	0.0,	2.0,	0.0,	0.0},
    {-2.0,	0.0,	1.0,	2.0,	2.0},
    {0.0,	0.0,	0.0,	2.0,	0.0},
    {-2.0,	0.0,	0.0,	2.0,	0.0},
    {0.0,	0.0,	-1.0,	2.0,	1.0},
    {0.0,	2.0,	0.0,	0.0,	0.0},
    {2.0,	0.0,	-1.0,	0.0,	1.0},
    {-2.0,	2.0,	0.0,	2.0,	2.0},
    {0.0,	1.0,	0.0,	0.0,	1.0},
    {-2.0,	0.0,	1.0,	0.0,	1.0},
    {0.0,	-1.0,	0.0,	0.0,	1.0},
    {0.0,	0.0,	2.0,	-2.0,	0.0},
    {2.0,	0.0,	-1.0,	2.0,	1.0},
    {2.0,	0.0,	1.0,	2.0,	2.0},
    {0.0,	1.0,	0.0,	2.0,	2.0},
    {-2.0,	1.0,	1.0,	0.0,	0.0},
    {0.0,	-1.0,	0.0,	2.0,	2.0},
    {2.0,	0.0,	0.0,	2.0,	1.0},
    {2.0,	0.0,	1.0,	0.0,	0.0},
    {-2.0,	0.0,	2.0,	2.0,	2.0},
    {-2.0,	0.0,	1.0,	2.0,	1.0},
    {2.0,	0.0,	-2.0,	0.0,	1.0},
    {2.0,	0.0,	0.0,	0.0,	1.0},
    {0.0,	-1.0,	1.0,	0.0,	0.0},
    {-2.0,	-1.0,	0.0,	2.0,	1.0},
    {-2.0,	0.0,	0.0,	0.0,	1.0},
    {0.0,	0.0,	2.0,	2.0,	1.0},
    {-2.0,	0.0,	2.0,	0.0,	1.0},
    {-2.0,	1.0,	0.0,	2.0,	1.0},
    {0.0,	0.0,	1.0,	-2.0,	0.0},
    {-1.0,	0.0,	1.0,	0.0,	0.0},
    {-2.0,	1.0,	0.0,	0.0,	0.0},
    {1.0,	0.0,	0.0,	0.0,	0.0},
    {0.0,	0.0,	1.0,	2.0,	0.0},
    {0.0,	0.0,	-2.0,	2.0,	2.0},
    {-1.0,	-1.0,	1.0,	0.0,	0.0},
    {0.0,	1.0,	1.0,	0.0,	0.0},
    {0.0,	-1.0,	1.0,	2.0,	2.0},
    {2.0,	-1.0,	-1.0,	2.0,	2.0},
    {0.0,	0.0,	3.0,	2.0,	2.0},
    {2.0,	-1.0,	0.0,	2.0,	2.0}};

static const struct nutation_coefficients coefficients[NUTATION_TERMS] = {
    {-171996.0,	-174.2,	92025.0,8.9},
    {-13187.0,	-1.6,  	5736.0,	-3.1},
    {-2274.0, 	 0.2,  	977.0,	-0.5},
    {2062.0,   	0.2,    -895.0,    0.5},
    {1426.0,    -3.4,    54.0,    -0.1},
    {712.0,    0.1,    -7.0,    0.0},
    {-517.0,    1.2,    224.0,    -0.6},
    {-386.0,    -0.4,    200.0,    0.0},
    {-301.0,    0.0,    129.0,    -0.1},
    {217.0,    -0.5,    -95.0,    0.3},
    {-158.0,    0.0,    0.0,    0.0},
    {129.0,	0.1,	-70.0,	0.0},
    {123.0,	0.0,	-53.0,	0.0},
    {63.0,	0.0,	0.0,	0.0},
    {63.0,	1.0,	-33.0,	0.0},
    {-59.0,	0.0,	26.0,	0.0},
    {-58.0,	-0.1,	32.0,	0.0},
    {-51.0,	0.0,	27.0,	0.0},
    {48.0,	0.0,	0.0,	0.0},
    {46.0,	0.0,	-24.0,	0.0},
    {-38.0,	0.0,	16.0,	0.0},
    {-31.0,	0.0,	13.0,	0.0},
    {29.0,	0.0,	0.0,	0.0},
    {29.0,	0.0,	-12.0,	0.0},
    {26.0,	0.0,	0.0,	0.0},
    {-22.0,	0.0,	0.0,	0.0},
    {21.0,	0.0,	-10.0,	0.0},
    {17.0,	-0.1,	0.0,	0.0},
    {16.0,	0.0,	-8.0,	0.0},
    {-16.0,	0.1,	7.0,	0.0},
    {-15.0,	0.0,	9.0,	0.0},
    {-13.0,	0.0,	7.0,	0.0},
    {-12.0,	0.0,	6.0,	0.0},
    {11.0,	0.0,	0.0,	0.0},
    {-10.0,	0.0,	5.0,	0.0},
    {-8.0,	0.0,	3.0,	0.0},
    {7.0,	0.0,	-3.0,	0.0},
    {-7.0,	0.0,	0.0,	0.0},
    {-7.0,	0.0,	3.0,	0.0},
    {-7.0,	0.0,	3.0,	0.0},
    {6.0,	0.0,	0.0,	0.0},
    {6.0,	0.0,	-3.0,	0.0},
    {6.0,	0.0,	-3.0,	0.0},
    {-6.0,	0.0,	3.0,	0.0},
    {-6.0,	0.0,	3.0,	0.0},
    {5.0,	0.0,	0.0,	0.0},
    {-5.0,	0.0,	3.0,	0.0},
    {-5.0,	0.0,	3.0,	0.0},
    {-5.0,	0.0,	3.0,	0.0},
    {4.0,	0.0,	0.0,	0.0},
    {4.0,	0.0,	0.0,	0.0},
    {4.0,	0.0,	0.0,	0.0},
    {-4.0,	0.0,	0.0,	0.0},
    {-4.0,	0.0,	0.0,	0.0},
    {-4.0,	0.0,	0.0,	0.0},
    {3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0},
    {-3.0,	0.0,	0.0,	0.0}};


long double Nutation::c_JD=0.0;
long double Nutation::c_longitude=0.0;
long double Nutation::c_obliquity=0.0;
long double Nutation::c_ecliptic=0.0;


Nutation::Nutation(const Time& t)
{
	long double D,M,MM,F,O,T,T2,T3,JD,JDE;
	long double coeff_sine, coeff_cos;
	int i;

   JD=t.JD();
	/* should we bother recalculating nutation */
	if (fabs(JD - c_JD) > NUTATION_EPOCH_THRESHOLD) {
		/* set the new epoch */
		c_JD = JD;

		/* set ecliptic */
		c_ecliptic = 23.0 + 26.0 / 60.0 + 27.407 / 3600.0;
		
		/* get julian ephemeris day */
//		JDE = ln_get_jde (JD);
		JDE = t.JDE();
		
		/* calc T */
		T = (JDE - 2451545.0)/36525;
		T2 = T * T;
		T3 = T2 * T;

		/* calculate D,M,M',F and Omega */
		D = 297.85036 + 445267.111480 * T - 0.0019142 * T2 + T3 / 189474.0;
		M = 357.52772 + 35999.050340 * T - 0.0001603 * T2 - T3 / 300000.0;
		MM = 134.96298 + 477198.867398 * T + 0.0086972 * T2 + T3 / 56250.0;
		F = 93.2719100 + 483202.017538 * T - 0.0036825 * T2 + T3 / 327270.0;
		O = 125.04452 - 1934.136261 * T + 0.0020708 * T2 + T3 / 450000.0;
	
		/* convert to radians */
		D = DegToRad (D);
		M = DegToRad (M);
		MM = DegToRad (MM);
		F = DegToRad (F);
		O = DegToRad (O);

		/* calc sum of terms in table 21A */
		for (i=0; i< NUTATION_TERMS; i++) {
			/* calc coefficients of sine and cosine */
			coeff_sine = (coefficients[i].longitude1 + (coefficients[i].longitude2 * T));
			coeff_cos = (coefficients[i].obliquity1 + (coefficients[i].obliquity2 * T));
			
			/* sum the arguments */
			if (arguments[i].D != 0) {
				c_longitude += coeff_sine * (sin (arguments[i].D * D));
				c_obliquity += coeff_cos * (cos (arguments[i].D * D));
			}
			if (arguments[i].M != 0) {
				c_longitude += coeff_sine * (sin (arguments[i].M * M));
				c_obliquity += coeff_cos * (cos (arguments[i].M * M));
			}
			if (arguments[i].MM != 0) {
				c_longitude += coeff_sine * (sin (arguments[i].MM * MM));
				c_obliquity += coeff_cos * (cos (arguments[i].MM * MM));
			}
			if (arguments[i].F != 0) {
				c_longitude += coeff_sine * (sin (arguments[i].F * F));
				c_obliquity += coeff_cos * (cos (arguments[i].F * F));
			}
			if (arguments[i].O != 0) {
				c_longitude += coeff_sine * (sin (arguments[i].O * O));
				c_obliquity += coeff_cos * (cos (arguments[i].O * O));
			}
		}    

		/* change to arcsecs */
		c_longitude /= 10000;
		c_obliquity /= 10000;

		/* change to degrees */
		c_longitude /= (60 * 60);
		c_obliquity /= (60 * 60);
		c_ecliptic += c_obliquity;
	}

	/* convert to radians and store results */
   _longitude = DegToRad(c_longitude);
   _obliquity = DegToRad(c_obliquity);
   _ecliptic = DegToRad(c_ecliptic);

}





/*
 *
 *
 * Definitions for dynamical_time_diff
 *
 *
 *
 */

#define DELTA_T_TERMS 192

struct year_TD
{
    int year;
	double TD;
};

/* Stephenson and Houlden  for years prior to 948 A.D.*/
static double get_dynamical_diff_sh1 (double JD);

/* Stephenson and Houlden  for years between 948 A.D. and 1600 A.D.*/
static double get_dynamical_diff_sh2 (double JD);

/* Table 9.a pg 72 for years 1620..1992.*/
static double get_dynamical_diff_table (double JD);

/* get the dynamical time diff in the near past / future 1992 .. 2010 */
static double get_dynamical_diff_near (double JD);

/* uses equation 9.1 pg 73 to calc JDE for othe JD values */          
static double get_dynamical_diff_other (double JD);


/* dynamical time in seconds for every second year from 1620 to 1992 */

static const double delta_t[DELTA_T_TERMS] =
{   124.0, 115.0, 106.0, 98.0, 91.0,
    85.0, 79.0, 74.0, 70.0, 65.0,
    62.0, 58.0, 55.0, 53.0, 50.0,
    48.0, 46.0, 44.0, 42.0, 40.0,
    37.0, 35.0, 33.0, 31.0, 28.0,
    26.0, 24.0, 22.0, 20.0, 18.0,
    16.0, 14.0, 13.0, 12.0, 11.0,
    10.0, 9.0, 9.0, 9.0, 9.0,
    9.0, 9.0, 9.0, 9.0, 10.0,
    10.0, 10.0, 10.0, 10.0, 11.0,
    11.0, 11.0, 11.0, 11.0, 11.0,
    11.0, 12.0, 12.0, 12.0, 12.0,
    12.0, 12.0, 13.0, 13.0, 13.0,
    13.0, 14.0, 14.0, 14.0, 15.0,
    15.0, 15.0, 15.0, 16.0, 16.0,
    16.0, 16.0, 16.0, 17.0, 17.0,
    17.0, 17.0, 17.0, 17.0, 17.0,
    17.0, 16.0, 16.0, 15.0, 14.0,
    13.7, 13.1, 12.7, 12.5, 12.5,
    12.5, 12.5, 12.5, 12.5, 12.3,
    12.0, 11.4, 10.6, 9.6, 8.6,
    7.5, 6.6, 6.0, 5.7, 5.6,
    5.7, 5.9, 6.2, 6.5, 6.8,
    7.1, 7.3, 7.5, 7.7, 7.8,
    7.9, 7.5, 6.4, 5.4, 2.9,
    1.6, -1.0, -2.7, -3.6, -4.7,
    -5.4, -5.2, -5.5, -5.6, -5.8,
    -5.9, -6.2, -6.4, -6.1, -4.7,
    -2.7, 0.0, 2.6, 5.4, 7.7,
    10.5, 13.4, 16.0, 18.2, 20.2,
    21.2, 22.4, 23.5, 23.9, 24.3,
    24.0, 23.9, 23.9, 23.7, 24.0,
    24.3, 25.3, 26.2, 27.3, 28.2,
    29.1, 30.0, 30.7, 31.4, 32.2,
    33.1, 34.0, 35.0, 36.5, 38.3,
    40.2, 42.2, 44.5, 46.5, 48.5,
    50.5, 52.2, 53.8, 54.9, 55.8,
    56.9, 58.3
    };
 					

/* Stephenson and Houlden  for years prior to 948 A.D.*/

static double get_dynamical_diff_sh1 (double JD)
{
    double TD,E;
    
    /* number of centuries from 948 */
    E = (JD - 2067314.5) / 36525.0;
    
    TD = 1830.0 - 405.0 * E + 46.5 * E * E;
    return (TD);
}

/* Stephenson and Houlden  for years between 948 A.D. and 1600 A.D.*/

static double get_dynamical_diff_sh2 (double JD)
{
    double TD,t;
    
    /* number of centuries from 1850 */
    t = (JD - 2396758.5) / 36525.0;
    
    TD = 22.5 * t * t;
    return TD;
}

/* Table 9.a pg 72 for years 1600..1992.*/
/* uses interpolation formula 3.3 on pg 25 */

static double get_dynamical_diff_table (double JD)
{
    double TD = 0;
    double a,b,c,n;
    int i;
    
    /* get no days since 1620 and divide by 2 years */
    i = (int)((JD - 2312752.5) / 730.5);
    
    /* get the base interpolation factor in the table */
    if (i > (DELTA_T_TERMS - 2))
        i = DELTA_T_TERMS - 2;
	
	/* calc a,b,c,n */
	a = delta_t[i] - delta_t[i+1];
	b = delta_t[i+1] - delta_t[i+2];
	c = a - b;
	n = ((JD - (2312752.5 + (730.5 * (i+1)))) / 730.5);
	
	TD = delta_t[i+1] + n / 2 * (-a - b + n * c);

   return TD;
}




/* get the dynamical time diff in the near past / future 1992 .. 2010 */
/* uses interpolation formula 3.3 on pg 25 */

static double get_dynamical_diff_near (double JD)
{
    double TD = 0;
    /* TD for 1990, 2000, 2010 */

//    double delta_T[3] = {56.9, 67.0 ,80.0}; // old

/* new: taken from measured and predicted deltaT-values
 * (from Earth Orientation Department of the US Naval Observatory)
 */
    double delta_T[3] = {56.86, 63.83 ,70.0};
    double a,b,c,n;
            
    /* calculate TD by interpolating value */
    a = delta_T[0] - delta_T[1];
    b = delta_T[1] - delta_T[2];
    c = a - b;
    
   /* get number of days since 2000 and divide by 10 years */
	n = (JD - 2451544.5) / 3652.5;
	TD = delta_T[1] + n / 2 * (-a - b + n * c);
	       
   return TD;
} 

/* uses equation 9.1 pg 73 to calc JDE for other JD values */          
static double get_dynamical_diff_other (double JD)
{     
    double TD;
    double a;
    
    a = (JD - 2382148);
    a *= a;

    TD = -15 + a / 41048480;
       
    return (TD);
}  


/*! \fn double get_dynamical_time_diff (long double JD)
* \param JD Julian Day
* \return TD
*
* Calculates the dynamical time (TD) difference in seconds (delta T) from 
* universal time.
*/
/* Equation 9.1 on pg 73.
*/

double get_dynamical_time_diff (long double JD)
{
   double TD;
   /* check when JD is, and use corresponding formula */
   /* check for date < 948 A.D. */
   if ( JD < 2067314.5 )
        /* Stephenson and Houlden */
	    TD = get_dynamical_diff_sh1 (JD);
   else if ( JD >= 2067314.5  && JD < 2305447.5 )
	    /* check for date 948..1600 A.D. Stephenson and Houlden */
    	TD = get_dynamical_diff_sh2 (JD);
	else if ( JD >= 2312752.5 && JD < 2448622.5 )
		/* check for value in table 1620..1992  interpolation of table */
		TD = get_dynamical_diff_table (JD);
	else if ( JD >= 2448622.5 && JD <= 2455197.5 )   
		/* check for near future 1992..2010 interpolation */
		TD = get_dynamical_diff_near (JD);       
	else
	    /* other time period outside */
	    TD = get_dynamical_diff_other (JD);   	    
		    
	return TD;
}




} // namespace hgz
