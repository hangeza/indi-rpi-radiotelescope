#ifndef _BASIC_H
#define _BASIC_H

#include <cmath>
#include <cfloat>
#include <complex>
#include <vector>
#include <valarray>
#include <iostream>
#include <algorithm>
#include <errno.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <numeric>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


namespace hgz
{

//#define LDOUBLE
//#define FLOAT

#ifdef LDOUBLE
 typedef long double real_t;
// typedef std::complex<real_t> complex_t;
 #define POSMAX    (real_t)LDBL_MAX
 #define POSMIN    (real_t)LDBL_MIN
 #define MACH_EPS  (real_t)LDBL_EPSILON
#else
#ifdef FLOAT
 typedef float real_t;
 #define POSMAX    (real_t)FLT_MAX
 #define POSMIN    (real_t)FLT_MIN
 #define MACH_EPS  (real_t)FLT_EPSILON
#else
 typedef double real_t;
 #define POSMAX    (real_t)DBL_MAX
 #define POSMIN    (real_t)DBL_MIN
 #define MACH_EPS  (real_t)DBL_EPSILON
#endif
#endif

typedef std::complex<real_t> complex_t;

namespace UNITS {
 const real_t TERA = 1e+12;
 const real_t GIGA = 1e+9;
 const real_t MEGA = 1e+6;
 const real_t KILO = 1000.;
 const real_t CENTI = 0.01;
 const real_t MILLI = 0.001;
 const real_t MICRO = 1e-6;
 const real_t NANO = 1e-9;
 const real_t PICO = 1e-12;
}

//#define pi        (double_t)3.1415926535897932384626433832795029L
#define pi        (real_t)M_PIl
#define twopi     (real_t)(pi*2.0)
#define pi2       (real_t) twopi
//#define POSITIVE  (real_t)1.0
//#define NEGATIVE  (real_t)-1.0

#define ZERO      (real_t)0.0
//#define ONE       (real_t)1.0
#define TWO       (real_t)2.0
#define HALF      (real_t)0.50


#define DToR  (pi/180.0)
#define HToR  (pi/12.0)
#define RToD  (180.0/pi)
#define RToH  (12.0/pi)
#define SToR  (DToR/3600.0)
#define DToH  (DToR*RToH)


// function pointers, simple
typedef double (*func1r)(double);
typedef double (*func2r)(double,double);
typedef double (*func3r)(double,double,double);
typedef complex_t (*func1c)(complex_t);
typedef complex_t (*func2c)(complex_t,complex_t);
typedef complex_t (*func3c)(complex_t,complex_t,complex_t);

// function pointers, parametric
typedef double (*func1r_p)(double,std::vector<double>);
typedef double (*func2r_p)(double,double,std::vector<double>);
typedef double (*func3r_p)(double,double,double,std::vector<double>);
typedef complex_t (*func1c_p)(complex_t,std::vector<double>);
typedef complex_t (*func2c_p)(complex_t,complex_t,std::vector<complex_t>);
typedef complex_t (*func3c_p)(complex_t,complex_t,complex_t,std::vector<complex_t>);



/*
template <class T>
class STLArray : public std::valarray<T>
{
   public:
      int xsize;
      int ysize;


   public:
      STLArray(){xsize=0;ysize=0;}
      //STLArray():std::valarray<T>() {}

      STLArray(size_t a_xsize, size_t a_ysize)
         :std::valarray<T>(a_xsize*a_ysize), xsize(a_xsize), ysize(a_ysize)
      {}

      STLArray(size_t a_xsize, size_t a_ysize, const T& def)
         :std::valarray<T>(def,a_xsize*a_ysize), xsize(a_xsize), ysize(a_ysize)
      {}


   private:
      struct Proxy
      {
         Proxy(long a_offset, signed int a_ysize, STLArray<T>* a_array):
            offset(a_offset), ysize(a_ysize), array(a_array)
         {}
         T & operator[](signed int j)
         {
            int i=j;
            if (j<0) i+=ysize;
            return array->at(offset + i);
         }
         private:
            long offset;
            signed int ysize;
            STLArray<T>* array;
      };

   public:
      Proxy operator[](signed int i)
      {
         int j=i;
         if (i<0) j+=xsize;
         long temp=(long)ysize*(long)j;
         return Proxy(temp,ysize, *this);
      }
};


template <class T>
std::ostream& operator<<(std::ostream& o, const STLArray<T>& v)
{
   for (int j=0;j<v.ysize;j++)
   {
      for (int i=0;i<v.xsize;i++)
         o<<v[i][j]<<" ";
      std::cout<<std::endl;
   }
   return o;
}
*/


template <class T>
class AbstractArray
{

   public:
      AbstractArray()
      {
         _size = 0;
         mem=0;
//         std::cout<<"ctor AbstractArray"<<std::endl;
      }

      AbstractArray(size_t a_size, const T& def):_size(a_size)
      {
         if (_size!=0)
            mem=new T[_size];
         else mem=0;
         for (size_t i=0;i<_size;i++) mem[i]=T(def);

//         std::cout<<"ctor AbstractArray(size_t,def): "<<_size<<" , "<<def<<std::endl;
      }

      AbstractArray(size_t a_size):_size(a_size)
      {

//         std::cout<<"ctor AbstractArray(size_t): "<<_size<<std::endl;

         if (_size!=0)
            mem=new T[_size];
         for (size_t i=0;i<_size;i++) mem[i]=T();

//         std::cout<<"ctor AbstractArray(size_t): "<<_size<<std::endl;

      }

   public:

      ~AbstractArray()
      {
//         std::cout<<"dtor AbstractArray"<<std::endl;
//         std::cout<<"size:"<<_size<<std::endl;
         if (_size)
            delete[] mem;
         _size = 0;
      }

      T sum() const
      {
         T sum=T();
         for (size_t i=0;i<_size;i++) sum+=mem[i];
         return sum;
      }

      T min() const
      {
         if (!_size) return T();
         //return *(std::min_element(mem,mem+_size));
         T min;
         size_t i=0;
#if __cplusplus > 199711L
	 while (i<_size && !std::isnormal(mem[i++]));
#else
	 while (i<_size && !std::isnormal<T>(mem[i++]));
#endif	    
         min=mem[i-1];
         std::less<T> less;
         for (;i<_size;i++)
         {
#if __cplusplus > 199711L
            if (!std::isnormal(mem[i])) continue;	   
#else
            if (!std::isnormal<T>(mem[i])) continue;	   
#endif	    
	    if (less(mem[i],min)) min=mem[i];
         }
         return min;
      }

      T max() const
      {
         if (!_size) return T();
         //return *(std::max_element(mem,mem+_size));
         size_t i=0;
         T max;
#if __cplusplus > 199711L
         while (i<_size && !std::isnormal(mem[i++]));
#else
         while (i<_size && !std::isnormal<T>(mem[i++]));
#endif	    
         max=mem[i-1];
         std::greater<T> greater;
         for (;i<_size;i++)
         {
#if __cplusplus > 199711L
            if (!std::isnormal(mem[i])) continue;	   
#else
            if (!std::isnormal<T>(mem[i])) continue;	   
#endif	    
	    if (greater(mem[i],max)) max=mem[i];
         }
         return max;
      }

      size_t size() const
      {
         return _size;
      }

      int typesize() const
      {
         return sizeof(T);
      }
   public:
//      int type_size;
   protected:
      size_t _size;
      T* mem;
};



double Gauss(double x);
double Gauss(double x, double mean, double width=1.);
double Gauss(double x, const std::vector<double>& params);
double Gauss2D(double x, double y);
double Gauss2D(double x, double y, double mean_x, double mean_y, double width_x,double width_y);

double Integral(double a, double b, func1r f);
double Integral2D(double a_x, double b_x, double a_y, double b_y, func2r f);


inline double sqr(double x) {return x*x;}

double modpi2(double x);
double modpi(double x);

unsigned long fak(int n);
long double fak(double n);
long long fibonacci(long n);
long sum_int(long n);
int basis(void);        /* Basis der Zahlendarstellung bestimmen */
real_t mach_eps(void);  /* Maschinengenauigkeit bestimmen */
real_t pos_min(void);   /* kleinste positive Gleitkommazahl bestimmen */
unsigned long bincoeff(int n, int k);



//const Function<double,double> Gauss(Gauss);


} // namespace hgz


#endif //_BASIC_H
