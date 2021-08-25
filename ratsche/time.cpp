//#include <cstdio>
//#include <typeinfo>
//#include <climits>
//#include <cassert>
#include <iostream>
//#include <ctime>
#include <sys/time.h>
#include <iomanip>

#include "time.h"
//#include "basic.h"
#include "astro.h"

namespace hgz{

Time::Time()
{
//			GetActualTime();
	_timestamp.tv_sec=0;
	_timestamp.tv_usec=0;
   tzset();
   _timezone=-::timezone/3600;
}

Time::Time(long double t)
{
	_timestamp.tv_sec=(time_t)t;
	_timestamp.tv_usec=(time_t)((t-_timestamp.tv_sec)*1e+6);
   tzset();
   _timezone=-::timezone/3600;
}

Time::Time(const Time& t)
{
	_timestamp.tv_sec=t._timestamp.tv_sec;
	_timestamp.tv_usec=t._timestamp.tv_usec;
   _timezone=t._timezone;
}

Time::Time(int y, int mon, int d, int h, int min, double sec)
{
   tm t;
   t.tm_year=y-1900;
   t.tm_mon=mon-1;
   t.tm_mday=d;
   t.tm_hour=h;
   t.tm_min=min;
   t.tm_sec=int(sec);
   _timestamp.tv_sec=mktime(&t);
   tzset();
   _timezone=-::timezone/3600;
  
_timestamp.tv_usec=(time_t)((double(sec)-(double)t.tm_sec)*1e+6);
}

Time::Time(int y, int mon, double d)
{
   tm t;
   t.tm_year=y-1900;
   t.tm_mon=mon-1;
   t.tm_mday=(int)d;
   double dd=d-(double)t.tm_mday;
   dd*=24;   
   t.tm_hour=(int)dd;

   dd=dd-(double)t.tm_hour;
   dd*=60;
   t.tm_min=(int)dd;

   dd=dd-(double)t.tm_min;
   dd*=60;
   t.tm_sec=int(dd);
   dd=dd-(double)t.tm_sec;

   _timestamp.tv_sec=mktime(&t);
   tzset();
   _timezone=-::timezone/3600;
   _timestamp.tv_usec=(time_t)(dd*1e+9);
}

long Time::n_sec() const
{
	return _timestamp.tv_usec*1000;
}

long Time::u_sec() const
{
	return _timestamp.tv_usec;
}

int Time::sec() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_sec;
}

int Time::min() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_min;
}

int Time::hour() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_hour;
}

int Time::day() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_mday;
}

int Time::month() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_mon+1;
}

int Time::year() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_year+1900;
}

int Time::wday() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_wday;
}

int Time::yday() const
{
	tm *t;
	t=localtime(&_timestamp.tv_sec);
	return t->tm_yday;
}

long double Time::JD() const
{
   long double _JD=2440587.5; // JD of 01/01/1970
   
   _JD+=((long double)(_timestamp.tv_sec)+(long double)(_timestamp.tv_usec)*1e-6)
         /(86400.);
   
   return _JD;
   
   int m=month();
   int y=year();
   double d=day();
   d+=double(hour()-_timezone)/24.+(double)min()/(24.*60.)
            +(double)sec()/(24.*3600.);
   
   std::cout<<"y="<<y<<" m="<<m<<" d="<<d<<std::endl;
   if (m<3)
   {
      m+=12;
      y-=1;
   }
   std::cout<<"y="<<y<<" m="<<m<<" d="<<d<<std::endl;
   return (long)(365.25*(y+4716))
         +(long)(30.6000000000001*(m+1))
         -(y/100)+(y/400)
         +d+2-1524.5;
//   return 365.25*(y+4716)+30.60001*(m+1)+d+2.-(y/100)+(y/400)-1524.5;
//   return d+(153*m-457)/5+365*y+y/4-y/100+y/400+1721118.5;
}

long double Time::JDE() const
{
   const double sec_per_day = 86400.;
   return (JD()+get_dynamical_time_diff(JD())/sec_per_day);
}

double Time::MeanSidereal() const
{
    long double sidereal;
    long double T;
    long double _JD;
    
    _JD=JD();
//    return ln_get_mean_sidereal_time(_JD);
    
    T = (_JD - 2451545.0) / 36525.0;
        
    /* calc mean angle */
    sidereal = 280.46061837 + (360.98564736629 * (_JD - 2451545.0)) + (0.000387933 * T * T) - (T * T * T / 38710000.0);
    
    /* add a convenient multiple of 360 degrees */
    sidereal /= 360.;
    sidereal *= twopi;
    sidereal = modpi2(sidereal);
    
    /* change to hours */
    sidereal *= 24.0 / twopi;
        
    return sidereal;
}

double Time::ApparentSidereal() const
{
//    return ln_get_apparent_sidereal_time(JD());

   /* get the mean sidereal time */
   long double sidereal = MeanSidereal();

   /* add corrections for nutation in longitude and for the true obliquity of 
   the ecliptic */   
   Nutation nut(*this);
   long double correction = (nut.Longitude() / 15.0 * cos (nut.Obliquity()));
   /* value is in radians so change it to hours and add to mean sidereal time */
   correction *= 24./twopi;
   sidereal += correction;
   return sidereal;
}

long double Time::timestamp() const
{
   return _timestamp.tv_sec+(long double)(_timestamp.tv_usec)*1e-6;
}

long double Time::operator()() const
{
   return timestamp();
//   return _time.tv_sec+(long double)(_time.tv_nsec)*1e-9;
}

Time& Time::operator+=(int seconds)
{
   _timestamp.tv_sec+=seconds;
   return *this;
}

Time& Time::operator-=(int seconds)
{
   _timestamp.tv_sec-=seconds;
   return *this;
}

Time& Time::operator+=(double seconds)
{
   long double t=timestamp()+seconds;

   _timestamp.tv_sec=(time_t)t;
   _timestamp.tv_usec=(time_t)(1e+6*(double)(t-(time_t)t));
   return *this;
}

Time& Time::operator-=(double seconds)
{
   long double t=timestamp()-seconds;

   _timestamp.tv_sec=(time_t)t;
   _timestamp.tv_usec=(time_t)(1e+6*(double)(t-(time_t)t));
   return *this;
}

std::ostream& operator<<(std::ostream& o, const Time &t)
{
//	std::cout.fill('0');
   o.fill('0');
   o<<std::setw(2)<<t.year()<<"/"
    <<std::setw(2)<<t.month()<<"/"
    <<std::setw(2)<<t.day();
    
   o<<" ";
    
   o<<std::setw(2)<<t.hour()<<":"
    <<std::setw(2)<<t.min()<<":"
    <<std::setw(2)<<t.sec()<<"."
    <<std::setw(9)<<t.n_sec();
    
	return o;
//	o<<ctime(&t._time.tv_sec)<<"\b";
}

std::istream& operator>>(std::istream& is, Time &t)
{
   int y;
   char ch;
   is>>y;
   is>>ch;
   int mon;
   is>>mon;
   is>>ch;
   int d;
   is>>d;
//         std::cout<<y<<ch<<mon<<ch<<d<<" ";
   int h;
   is>>h;
   is>>ch;
   int m;
   is>>m;
   is>>ch;
   double s;
   is>>s;
//         std::cout<<h<<ch<<m<<ch<<s<<std::endl;
   Time temp(y,mon,d,h,m,s);
   t=temp;
   return is;
}

Time Time::Now()
{
	Time t;
	t.GetActualTime();
	return t;
}

void Time::GetActualTime()
{
	gettimeofday(&_timestamp,0);
//   clock_gettime(CLOCK_REALTIME,&_time);
}

} // namespace hgz
