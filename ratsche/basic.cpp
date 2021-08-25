#include <cstdlib>
#include <complex>
#include <cmath>
#include <iostream>
#include <cassert>
//#include <algorithm>
#include <numeric>
#include "basic.h"
//#include "mixfft_c.h"
//#include "func.h"

using namespace std;
//using namespace hgz;

namespace hgz{


#define maxcol  255 | (255 << 8) | (255 << 16)

#define dm_none  maxcol
#define dm_rainbow  1280
#define dm_blackwhite  255


/* math. Functions */

double Gauss(double x)
{
   double y = 1.0/(sqrt(twopi));
   y*=exp(-x*x/2.0);
/*   std::cout<<" *** Gauss(double) called ***"<<std::endl;
*/
   return y;
}

double Gauss(double x, double mean, double width)
{
   double y = 1.0/(width*sqrt(twopi));
   y*=exp(-sqr(x-mean)/(2.0*sqr(width)));
/*   std::cout<<" *** Gauss(double, double, double) called ***"<<std::endl;
*/
   return y;
}

double Gauss(double x, const std::vector<double>& params)
{
   assert (params.size()==2);
   double y = 1.0/(params[1]*sqrt(twopi));
/*
   std::cout<<" *** Gauss(double, std::vector) called ***"<<std::endl;
   std::cout<<"     p.size()="<<params.size()<<std::endl;
   std::cout<<"     p[0]="<<params[0]<<" , p[1]="<<params[1]<<std::endl;
*/   
   y*=exp(-sqr(x-params[0])/(2.0*sqr(params[1])));
   return y;
}


double Gauss2D(double x, double y)
{
   double f = 1.0/(twopi);
   f*=exp(-x*x/2.0);
   f*=exp(-y*y/2.0);
   return f;
}

double Gauss2D(double x, double y, double mean_x, double mean_y, double width_x,double width_y)
{
   double f = 1.0/(width_x*width_y*twopi);
   f*=exp(-sqr(x-mean_x)/(2.0*sqr(width_x)));
   f*=exp(-sqr(y-mean_y)/(2.0*sqr(width_y)));
   return f;
}

template <int dims>
double Gauss(double x[])
{
   double f = 1.0/twopi;
   for (int i=0; i< dims; i++)
   {
      f*=exp(-sqr(x[i])/(2.0));
   }
   return f;
}


template <int dims>
double Gauss(double x[], double mean[], double sigma[])
{
   double f = 1.0/(std::accumulate(&sigma[0],&sigma[dims],1,std::multiplies<double>())*twopi);
   for (int i=0; i< dims; i++)
   {
      f*=exp(-sqr(x[i]-mean[i])/(2.0*sqr(sigma[i])));
   }
   return f;
}

/* Integration mittels Simpson-Regel */
/*
double Integral(double a, double b, func1r f)
{
   const int n=1024;
   double c=2.;
   double h=(b-a)/n;
   double simp=f(a)+f(b);
   for (int i=1;i<n;i++) simp+=(c=6-c)*f(a+i*h);
   return simp*h/3;
}
*/
/* Integration mittels Gauss-Verfahren (n=2)*/

double Integral(double a, double b, func1r f)
{
   const int N=1024;
   static const double a0=sqrt(0.6);
   double h=(b-a)/(2*N);
   double sum=0;
   for (int i=0;i<N;i++) sum+=5*f(a+(2*i+1)*h-a0*h)+8*f(a+(2*i+1)*h)+5*f(a+(2*i+1)*h+a0*h);
   return sum*h/9;
}


/*
double Integral(double a, double b, func1r f)
{
   unsigned long int n=1000;
   double h=1.;
//   double sum=INFINITY;
   double sum=1e+38;
   double sum_old=0;
   const static double eps=1e-10;
   
   while (std::abs(sum-sum_old)>eps && h>MACH_EPS)
   {
      h=(b-a)/n;
      sum_old=sum;
      sum=0;
      for (double x=a; x<b; x+=h)
      {
         sum+=f(x)*h;
      }
      n++;
      cout<<"n="<<n<<" sum="<<sum<<" eps="<<std::abs(sum-sum_old)<<" h="<<h<<endl;
   }
   
   return sum;
}
*/

/* Integration mittels Gauss-Verfahren */
double Integral2D(double a_x, double b_x, double a_y, double b_y, func2r f)
{
   const int Nx=256;
   const int Ny=256;
   static const double a0=sqrt(0.6);
   double hx=(b_x-a_x)/(2*Nx);
   double hy=(b_y-a_y)/(2*Ny);

   double sum=0;
   for (int i=0;i<Nx;i++)
      for (int k=0;k<Ny;k++)
      {   
         double xi=a_x+(2*i+1)*hx;
         double xk=a_y+(2*k+1)*hy;
         
         sum+= 25*( f(xi-a0*hx,xk-a0*hy)
                   +f(xi-a0*hx,xk+a0*hy)
                   +f(xi+a0*hx,xk-a0*hy)
                   +f(xi+a0*hx,xk+a0*hy))
              +40*( f(xi,xk-a0*hy)
                   +f(xi-a0*hx,xk)
                   +f(xi+a0*hx,xk)
                   +f(xi,xk+a0*hy))
              +64*f(xi,xk);
      }
   return sum*hx*hy/81;
}



double Convolution(func1r f1, func1r f2, double x, double x_start, double x_end)
{
   double f;
      

   return f;
}


/* Functions */

double modpi2(double x)
{
   double y=x;

/*
   cout<<"pi="<<pi<<endl;
   cout<<"pi2="<<pi2<<endl;
   cout<<"y="<<y<<endl;
   cout<<"y/pi2="<<y/pi2<<endl;
   cout<<"trunc(y/pi2)="<<trunc(y/pi2)<<endl;
*/

   y = y - trunc(y/pi2)*pi2;
   if (y<0) y+=pi2;
   return y;   
}

double modpi(double x)
{
   double y=x;

   y = y - trunc(y/pi2)*pi2;
   if (y>pi) y-=pi2;
   return y;   
}

unsigned long fak(int n)
{
   if (n<2) return 1UL;
   return n*fak(static_cast<int>(n-1));
}

long double fak(double n)
{
   if (n<2.0) return 1.0;
   return n*fak(static_cast<double>(n)-1.0);
}

long long fibonacci(long n)
{
   if (n==0) return 0;
   if (n==1) return 1;
   return fibonacci(n-2)+fibonacci(n-1);
}

long sum_int(long n)
{
   if (n==0) return 0;
   if (n==1) return 1;
   return n+sum_int(n-1);
}


int basis(void)              /* Basis der Zahlendarstellung bestimmen */

/***********************************************************************
* die Basis der Zahlendarstellung maschinenunabhaengig bestimmen,      *
* falls nicht schon in frueheren Aufrufen geschehen, und als           *
* Funktionswert zurueckgeben                                           *
*                                                                      *
* benutzte globale Namen:                                              *
* =======================                                              *
* REAL, ONE, TWO                                                       *
***********************************************************************/

{
  real_t x,
       eins,
       b;


  x = eins = b = 1.0;

  while ((x + eins) - x == eins)
    x *= 2.0;
  while ((x + b) == x)
    b *= 2.0;


  return (int)((x + b) - x);
}


static int groesser1(real_t x)         /* Hilfsfunktion fuer mach_eps() */

/***********************************************************************
* Hilfsfunktion fuer mach_eps() (noetig, um gewisse Compileroptimie-   *
* rungen zu umgehen): melden, ob die uebergebene Zahl x groesser als   *
* Eins ist                                                             *
*                                                                      *
* benutzte globale Namen:                                              *
* =======================                                              *
* REAL, ONE                                                            *
***********************************************************************/

{
  return ((real_t)x > (real_t)1.);
}



real_t mach_eps(void)                 /* Maschinengenauigkeit bestimmen */

/***********************************************************************
* die Maschinengenauigkeit maschinenunabhaengig bestimmen, falls nicht *
* schon in frueheren Aufrufen geschehen, und als Funktionswert         *
* zurueckgeben                                                         *
*                                                                      *
* benutzte globale Namen:                                              *
* =======================                                              *
* REAL, boolean, FALSE, ONE, HALF, TWO, TRUE                           *
***********************************************************************/

{
  static real_t    epsilon;
  static bool schon_berechnet = false;

  if (! schon_berechnet)
  {
    for (epsilon = 1.; groesser1((real_t)1. + epsilon); )
    {
//      std::cout<<epsilon<<std::endl;
      epsilon *= HALF;
    }
    epsilon         *= TWO;
    schon_berechnet  = true;
  }

  return epsilon;
}

real_t pos_min(void)       /* kleinste positive Gleitkommazahl bestimmen */

/***********************************************************************
* die kleinste positive Gleitkommazahl berechnen, falls nicht schon    *
* in frueheren Aufrufen geschehen, und als Funktionswert zurueckgeben. *
* Der Algorithmus besteht darin, dass in y der Anfangswert Eins so     *
* lange halbiert wird, bis er sich nicht mehr aendert oder zu Null     *
* wird. Damit dabei keine Endlosschleife entsteht, wurde ein Zaehler   *
* in die Iteration eingebaut, der nach 32767 Halbierungsversuchen auf  *
* jeden Fall fuer den Abbruch der Schleife sorgt.                      *
*                                                                      *
* benutzte globale Namen:                                              *
* =======================                                              *
* REAL, boolean, FALSE, ONE, TWO, ZERO, HALF, TRUE                     *
***********************************************************************/

{
  static real_t    y;  /* nach Schleifenende: kleinste Gleitkommazahl   */
  real_t           x;  /* in der Schleife:  2 * y (zum Vergleich mit y) */
  long             i = 0;  /* Zaehler zur Verhinderung einer Endlosschleife */
  static bool schon_berechnet = false;

  if (! schon_berechnet)
  {
/*
    for (i = 0, x = ONE, y = TWO; (x != ZERO) && (x != y) && (i < 32767); i++)
    {  
//      std::cout<<y<<std::endl;
   
//      temp=y;
      y =  x;
      x = x*HALF;
//      x = x/TWO;
    }
*/
   ostringstream s;
   x=(real_t)1.;
   y=(real_t)2.;
   while (++i<0xfffff)
   {
      s<<y<<std::endl; // Trick zur Umgehung der Compileroptimierung
      if (x==ZERO) break;
      if (x==y) break;
      y = x;
      x*=HALF;
   }
//   std::cout<<i<<std::endl;
    
    schon_berechnet = true;
  }

  return y;
}


unsigned long bincoeff(int n, int k)
{
   unsigned long bin = 1;
   assert(n>=k && k>=0);
   if (k>n/2) k=n-k;
   for (int i=0;i<k;i++) bin=bin*(n-i)/(i+1UL);
   return bin;
}

/*
// recursive - poor performance
unsigned long bincoeff(int n, int k)
{
   assert(n>=k && k>=0);
   if (k==0 || k==n) return 1UL;
   else return bincoeff(n-1,k-1)+bincoeff(n-1,k);
}
*/

} // namespace hgz
