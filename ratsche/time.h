#ifndef _HGZTIME_H
#define _HGZTIME_H

//#include <cstdio>
//#include <typeinfo>
//#include <climits>
//#include <cassert>
#include <iostream>
//#include <ctime>
//#include <string>
#include <sys/time.h>

namespace hgz{

//!general class to store, read and process time and date information
/*! this class is for easy access to date and time, serves as interface
 to system time functions and provides several time conversion functions.
 \author HG Zaunick
 
 */
class Time
{
	public:

    	//! default constructor
      /*! Creates Instance with date and time initialized to 
        01/01/1970 00:00 GMT
    	*/
		Time();

    	/*! Creates Instance initialized to \e secs seconds since 01/01/1970 00:00 GMT
    	*/
		Time(long double secs);

    	/*! Copy constructor
    	*/
      Time(const Time&);

    	/*! initialize instance with a given time
          \param year gregorian year
          \param month month of year
          \param day day of month
          \param hour hour of day (0..23)
          \param min minute (0..59)
          \param sec seconds with fraction
    	*/
		Time(int year, int month = 1, int day = 1,
			  int hour = 0, int min = 0, double sec = 0);

    	/*! initialize instance with a given time
			 \param year gregorian year
			 \param month month of year
			 \param day day with fraction
    	*/
      Time(int year, int month = 1, double day = 1.);

    	/*! default destructor
       */
		~Time(){}

      /*! Get the nanoseconds of the actual second \n
       * \note deprecated
       */
		long n_sec() const;

      /*! Get the microseconds of the actual second
       */
		long u_sec() const;
      /*! Get the seconds (of the minute)
       */
		int sec() const;

      /*! Get the minutes (of the hour)
       */
		int min() const;

      /*! Get the hours (of day)
       */
		int hour() const;

      /*! Get the days (of month)
       */
		int day() const;

      /*! Get the month (of year)
       */
      int month() const;

      /*! Get the year
       */
		int year() const;

      /*! Get the day of week
       */
		int wday() const;

      /*! Get the day of year
       */
		int yday() const;

      /*! Get and Set the timezone the time referes to \n
       the timezone is the difference in hours to GMT
       */
      int timezone() { return _timezone; }
//      std::string tz_name();

      /*! Calculate the Julian Date \n
          JD is the Nr. of Days since 01/01/4716 B.C.
          \return Julian Date
          \par Reference:
          J.Meeus: "Astronomical Calculations"
       */
      long double JD() const;
      
      /*! Calculate the Julian Ephemeris Date \n
          \return Julian Ephemeris Date
          \par Reference:
          J.Meeus: "Astronomical Calculations"
       */
      long double JDE() const;
      
      /*! Calculate the mean sidereal time at the meridian of Greenwich
      * \return Mean sidereal time in hours.
      *
      * \par Reference:
      *  Formula 11.1, 11.4 pg 83 
      */
      double MeanSidereal() const;
      
      /*! Calculate the apparent sidereal time at the meridian of Greenwich
      * \return Apparent sidereal time in hours.
      *
      * \par Reference:
      *  Formula ?, ? pg ? 
      */
      double ApparentSidereal() const;
      
      /*! Returns unique timestamp
      * \return timestamp in seconds since 01/01/1970
      * \note resolution of return value is one microsecond
      */
      long double timestamp() const;
      
      /*! Returns system time
      * \return date and time in system format
      * \note this function has the same result like timestamp
      */
      long double operator()() const;

      Time& operator+=(int seconds);
      Time& operator-=(int seconds);
      Time& operator+=(double seconds);
      Time& operator-=(double seconds);

		/*! ostream operator for class Time */
		friend std::ostream& operator<<(std::ostream& o, const Time &t);

		/*! istream operator for class Time */
      friend std::istream& operator>>(std::istream& is, Time &t);

      /*! Returns an instantiation of class Time which holds
         current (system) time and date
       */
		static Time Now();

	private:

      /*! Get current time and date and set members adequately
       */
      void GetActualTime();
	protected:
		struct timeval _timestamp;
      int _timezone;		
};

} // namespace hgz

#endif // _HGZTIME_H
