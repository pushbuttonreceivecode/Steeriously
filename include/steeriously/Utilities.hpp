#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cassert>
#include <iomanip>

namespace steer //namespace steeriously
{

///< Useful typedefs for signed/unsigned integers.
typedef signed   char Int8;
typedef unsigned char Uint8;
typedef signed   short Int16;
typedef unsigned short Uint16;
typedef signed   int Int32;
typedef unsigned int Uint32;
typedef signed   long long Int64;
typedef unsigned long long Uint64;

///< Useful constants for minimum and maximum values.
const int     MaxInt    = (std::numeric_limits<int>::max)();
const double  MaxDouble = (std::numeric_limits<double>::max)();
const double  MinDouble = (std::numeric_limits<double>::min)();
const float   MaxFloat  = (std::numeric_limits<float>::max)();
const float   MinFloat  = (std::numeric_limits<float>::min)();

///< Useful constants for Pi.
const float   Pi        = 3.14159f;
const float   TwoPi     = Pi * 2;
const float   HalfPi    = Pi / 2;
const float   QuarterPi = Pi / 4;

/**
    \fn template <typename T>
        inline bool isNaN(T val);
    \brief Returns true if the value is NaN
    \param dt - a plain old float.
**/
template <typename T>
inline bool isNaN(T val)
{
  return val != val;
}

/**
    \fn inline float DegsToRads(float degs);
    \brief Converts degrees to radians and returns the value.
    \param degrees - a plain old float.
**/
inline float DegreesToRadians(float degrees)
{
  return TwoPi * (degrees/360.f);
}

/**
    \fn inline bool IsZero(float val);
    \brief Returns true if the parameter is equal to zero.
    \param val - a plain old float.
**/
inline bool IsZero(float val)
{
  return ( (-MinFloat < val) && (val < MinFloat) );
}

/**
    \fn inline bool InRange(float start, float end, float val);
    \brief Returns true if the third parameter is in the range described by the first two parameters.
    \param start - a plain old float.
    \param end - a plain old float.
    \param val - a plain old float.
**/
inline bool InRange(float start, float end, float val)
{
  if (start < end)
  {
    if ( (val > start) && (val < end) ) return true;
    else return false;
  }

  else
  {
    if ( (val < start) && (val > end) ) return true;
    else return false;
  }
}

/**
    \fn template <class T>
        T Maximum(const T& v1, const T& v2);
    \brief Returns maximum value given two types.
    \param v1 - any type with operator > overload or primitive type.
    \param v2 - any type with operator > overload or primitive type.
**/
template <class T>
T Maximum(const T& v1, const T& v2)
{
  return v1 > v2 ? v1 : v2;
}

//----------------------------------------------------------------------------
//  some random number functions.
//----------------------------------------------------------------------------

/**
    \fn inline int RandInt(int x,int y);
    \brief Returns a random integer between x and y.
    \param x - a plain old int.
    \param y - a plain old int.
**/
inline int RandInt(int x,int y) {return rand()%(y-x+1)+x;}

/**
    \fn inline float RandFloat();
    \brief Returns a random float between 1 and the maximum value for a float.
**/
inline float RandFloat(){return ((rand())/(RAND_MAX+1.0));}

/**
    \fn inline float RandInRange(float x, float y);
    \brief Returns a random float between x and y.
    \param x - a plain old float.
    \param y - a plain old float.
**/
inline float RandInRange(float x, float y)
{
  return x + RandFloat()*(y-x);
}

/**
    \fn inline bool RandBool();
    \brief Returns true or false randomly.
**/
inline bool   RandBool()
{
  if (RandInt(0,1)) return true;

  else return false;
}

/**
    \fn inline float RandomClamped();
    \brief Returns a random float in the range -1 < n < 1.
**/
inline float RandomClamped(){return RandFloat() - RandFloat();}

/**
    \fn inline float RandGaussian(float mean = 0.0, float standard_deviation = 1.0);
    \brief Returns a random number with a normal distribution. See method at http://www.taygeta.com/random/gaussian.html.
    \param mean - a plain old float.
    \param standard_deviation - a plain old float.
**/
inline float RandGaussian(float mean = 0.0, float standard_deviation = 1.0)
{
	float x1, x2, w, y1;
	static float y2;
	static int use_last = 0;

	if (use_last)		        /* use value from previous call */
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do
        {
			x1 = 2.0 * RandFloat() - 1.0;
			x2 = 2.0 * RandFloat() - 1.0;
			w = x1 * x1 + x2 * x2;
		}
        while ( w >= 1.0 );

            w = sqrt( (-2.0 * log( w ) ) / w );
            y1 = x1 * w;
            y2 = x2 * w;
            use_last = 1;
	}

	return( mean + y1 * standard_deviation );
}

//-----------------------------------------------------------------------
//
//  some handy little functions
//-----------------------------------------------------------------------

/**
    \fn inline float Sigmoid(float input, float response = 1.0);
    \brief Returns a value from the sigmoid (S-curve) function based on input and response.
    \param input - a plain old float.
    \param response - a plain old float.
**/
inline float Sigmoid(float input, float response = 1.0)
{
	return ( 1.0 / ( 1.0 + exp(-input / response)));
}

/**
    \fn template <class T>
        inline T MaxOf(const T& a, const T& b);
    \brief Returns the maximum of two values.
    \param a - any type with > operator overload or a primitive type.
    \param b - any type with > operator overload or a primitive type.
**/
template <class T>
inline T MaxOf(const T& a, const T& b)
{
  if (a>b) return a; return b;
}

/**
    \fn template <class T>
        inline T MinOf(const T& a, const T& b);
    \brief Returns the minimum of two values.
    \param a - any type with > operator overload or a primitive type.
    \param b - any type with > operator overload or a primitive type.
**/
template <class T>
inline T MinOf(const T& a, const T& b)
{
  if (a<b) return a; return b;
}

/**
    \fn template <class T, class U, class V>
        inline void Clamp(T& arg, const U& minVal, const V& maxVal);
    \brief Clamps the first argument between the second two.
    \param arg - any type with > and < operator overloads or a primitive type.
    \param minVal - any type with > and < operator overloads or a primitive type.
    \param maxVal - any type with > and < operator overloads or a primitive type.
**/
template <class T, class U, class V>
inline void Clamp(T& arg, const U& minVal, const V& maxVal)
{
  assert ( (minVal < maxVal) && "<Clamp>MaxVal < MinVal!");

  if (arg < (T)minVal)
  {
    arg = (T)minVal;
  }

  if (arg > (T)maxVal)
  {
    arg = (T)maxVal;
  }
}

/**
    \fn inline int Rounded(float val);
    \brief Rounds a double up or down depending on its value.
    \param val - a plain old float.
**/
inline int Rounded(float val)
{
  int    integral = (int)val;
  float mantissa = val - integral;

  if (mantissa < 0.5)
  {
    return integral;
  }

  else
  {
    return integral + 1;
  }
}

/**
    \fn inline int RoundUnderOffset(float val, float offset);
    \brief rounds a double up or down depending on whether its mantissa is higher or lower than offset.
    \param val - a plain old float.
    \param offset - a plain old float.
**/
inline int RoundUnderOffset(float val, float offset)
{
  int    integral = (int)val;
  float mantissa = val - integral;

  if (mantissa < offset)
  {
    return integral;
  }

  else
  {
    return integral + 1;
  }
}

/**
    \fn inline bool isEqual(float a, float b);
    \brief Compares two real numbers. Returns true if they are equal.
    \param a - a plain old float.
    \param b - a plain old float.
**/
inline bool isEqual(float a, float b)
{
  if (fabs(a-b) < 1E-12)
  {
    return true;
  }

  return false;
}

/**
    \fn inline bool isEqual(double a, double b);
    \brief Compares two real numbers. Returns true if they are equal.
    \param a - a plain old double.
    \param b - a plain old double.
**/
inline bool isEqual(double a, double b)
{
  if (fabs(a-b) < 1E-12)
  {
    return true;
  }

  return false;
}

/**
    \fn template <class T>
        inline float Average(const std::vector<T>& v);
    \brief Finds the average for any std::vector of type T which can be cast to a float or any primitive type.
    \param v - a std::vector of type T.
**/
template <class T>
inline float Average(const std::vector<T>& v)
{
  float average = 0.0;

  for (unsigned int i=0; i < v.size(); ++i)
  {
    average += (float)v[i];
  }

  return average / (float)v.size();
}

/**
    \fn inline float StandardDeviation(const std::vector<float>& v);
    \brief Finds the standard deviation from a std::vector of floats.
    \param v - a std::vector of floats.
**/
inline float StandardDeviation(const std::vector<float>& v)
{
  float sd      = 0.0;
  float average = Average(v);

  for (unsigned int i=0; i<v.size(); ++i)
  {
    sd += (v[i] - average) * (v[i] - average);
  }

  sd = sd / v.size();

  return sqrt(sd);
}

/**
    \fn template <class container>
        inline void DeleteSTLContainer(container& c);
    \brief Frees memory in any STL container, with the exception of std::map.
    \param c - any STL container.
**/
template <class container>
inline void DeleteSTLContainer(container& c)
{
  for (typename container::iterator it = c.begin(); it!=c.end(); ++it)
  {
    delete *it;
    *it = NULL;
  }
}

/**
    \fn template <class map>
        inline void DeleteSTLMap(map& m);
    \brief Frees memory in a std::map conainer.
    \param m - a std::map.
**/
template <class map>
inline void DeleteSTLMap(map& m)
{
  for (typename map::iterator it = m.begin(); it!=m.end(); ++it)
  {
    delete it->second;
    it->second = NULL;
  }
}

} //namespace steeriously

#endif //UTILITIES_HPP

