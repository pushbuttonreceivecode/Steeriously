#ifndef VECTOR2_HPP
#define VECTOR2_HPP

#include <math.h>
#include <iosfwd>
#include <limits>

#include <steeriously/Utilities.hpp>

namespace steer
{


/**
*\struct Vector2
*\brief A 2D vector struct used in many steering calculations.
**/
struct Vector2
{
  double x;
  double y;

    /**
		* \fn Vector2();
		* \brief Default constructor
    **/
    Vector2():x(0.0),y(0.0){}
    Vector2(double a, double b):x(a),y(b){}

    //sets x and y to zero
    /**
		* \fn Zero();
		* \brief Set both x and y values to zero.
    **/
    void Zero(){x=0.0; y=0.0;}

    //returns true if both x and y are zero
    /**
		* \fn bool isZero() const;
		* \brief Returns true if both x and y values are zero.
    **/
    bool isZero() const {return (x*x + y*y) < MinDouble;}

    //returns the length of the vector
    /**
		* \fn inline double Length() const;
		* \brief Returns the length of the vector.
    **/
    inline double Length()const;

    //returns the squared length of the vector (thereby avoiding the sqrt)
    /**
		* \fn inline double LengthSq() const;
		* \brief Returns the squared length of the vector (thereby avoiding sqrt).
    **/
    inline double LengthSq() const;

    /**
		* \fn inline void Normalize();
		* \brief Normalizes the vector.
    **/
    inline void Normalize();

    /**
		* \fn inline double Dot(const Vector2& v2) const;
		* \brief Returns the dot product of the vector.
    **/
    inline double Dot(const Vector2& v2) const;

    //returns positive if v2 is clockwise of this vector,
    //negative if anticlockwise (assuming the Y axis is pointing down,
    //X axis to right like a Window app)
    /**
		* \fn inline int Sign(const Vector2& v2) const;
		* \brief Returns positive if v2 is clockwise of this vector, negative if anticlockwise (assuming y-axis is pointing down and x-axis is pointing right).
    **/
    inline int Sign(const Vector2& v2) const;

    //returns the vector that is perpendicular to this one.
    /**
		* \fn inline Vector2 Perp() const;
		* \brief Returns the vector that is perpendicular to this vector.
    **/
    inline Vector2 Perp() const;

  //adjusts x and y so that the length of the vector does not exceed max
  inline void      Truncate(double max);

  //returns the distance between this vector and th one passed as a parameter
  inline double    Distance(const Vector2 &v2)const;

  //squared version of above.
  inline double    DistanceSq(const Vector2 &v2)const;

  inline void      Reflect(const Vector2& norm);

  //returns the vector that is the reverse of this vector
  inline Vector2  GetReverse()const;


  //we need some overloaded operators
  const Vector2& operator+=(const Vector2 &rhs)
  {
    x += rhs.x;
    y += rhs.y;

    return *this;
  }

  const Vector2& operator-=(const Vector2 &rhs)
  {
    x -= rhs.x;
    y -= rhs.y;

    return *this;
  }

  const Vector2& operator*=(const double& rhs)
  {
    x *= rhs;
    y *= rhs;

    return *this;
  }

  const Vector2& operator/=(const double& rhs)
  {
    x /= rhs;
    y /= rhs;

    return *this;
  }

  bool operator==(const Vector2& rhs)const
  {
    return (isEqual(x, rhs.x) && isEqual(y,rhs.y) );
  }

  bool operator!=(const Vector2& rhs)const
  {
    return (x != rhs.x) || (y != rhs.y);
  }

};

//-----------------------------------------------------------------------some more operator overloads
inline Vector2 operator*(const Vector2 &lhs, double rhs);
inline Vector2 operator*(double lhs, const Vector2 &rhs);
inline Vector2 operator-(const Vector2 &lhs, const Vector2 &rhs);
inline Vector2 operator+(const Vector2 &lhs, const Vector2 &rhs);
inline Vector2 operator/(const Vector2 &lhs, double val);

//------------------------------------------------------------------------member functions

//------------------------- Length ---------------------------------------
//
//  returns the length of a 2D vector
//------------------------------------------------------------------------
inline double Vector2::Length()const
{
  return sqrt(x * x + y * y);
}


//------------------------- LengthSq -------------------------------------
//
//  returns the squared length of a 2D vector
//------------------------------------------------------------------------
inline double Vector2::LengthSq()const
{
  return (x * x + y * y);
}


//------------------------- Vec2DDot -------------------------------------
//
//  calculates the dot product
//------------------------------------------------------------------------
inline double Vector2::Dot(const Vector2 &v2)const
{
  return x*v2.x + y*v2.y;
}

//------------------------ Sign ------------------------------------------
//
//  returns positive if v2 is clockwise of this vector,
//  minus if anticlockwise (Y axis pointing down, X axis to right)
//------------------------------------------------------------------------
enum {clockwise = 1, anticlockwise = -1};

inline int Vector2::Sign(const Vector2& v2)const
{
  if (y*v2.x > x*v2.y)
  {
    return anticlockwise;
  }
  else
  {
    return clockwise;
  }
}

//------------------------------ Perp ------------------------------------
//
//  Returns a vector perpendicular to this vector
//------------------------------------------------------------------------
inline Vector2 Vector2::Perp()const
{
  return Vector2(-y, x);
}

//------------------------------ Distance --------------------------------
//
//  calculates the euclidean distance between two vectors
//------------------------------------------------------------------------
inline double Vector2::Distance(const Vector2 &v2)const
{
  double ySeparation = v2.y - y;
  double xSeparation = v2.x - x;

  return sqrt(ySeparation*ySeparation + xSeparation*xSeparation);
}


//------------------------------ DistanceSq ------------------------------
//
//  calculates the euclidean distance squared between two vectors
//------------------------------------------------------------------------
inline double Vector2::DistanceSq(const Vector2 &v2)const
{
  double ySeparation = v2.y - y;
  double xSeparation = v2.x - x;

  return ySeparation*ySeparation + xSeparation*xSeparation;
}

//----------------------------- Truncate ---------------------------------
//
//  truncates a vector so that its length does not exceed max
//------------------------------------------------------------------------
inline void Vector2::Truncate(double max)
{
  if (this->Length() > max)
  {
    this->Normalize();

    *this *= max;
  }
}

//--------------------------- Reflect ------------------------------------
//
//  given a normalized vector this method reflects the vector it
//  is operating upon. (like the path of a ball bouncing off a wall)
//------------------------------------------------------------------------
inline void Vector2::Reflect(const Vector2& norm)
{
  *this += 2.0 * this->Dot(norm) * norm.GetReverse();
}

//----------------------- GetReverse ----------------------------------------
//
//  returns the vector that is the reverse of this vector
//------------------------------------------------------------------------
inline Vector2 Vector2::GetReverse()const
{
  return Vector2(-this->x, -this->y);
}


//------------------------- Normalize ------------------------------------
//
//  normalizes a 2D Vector
//------------------------------------------------------------------------
inline void Vector2::Normalize()
{
  double vector_length = this->Length();

  if (vector_length > std::numeric_limits<double>::epsilon())
  {
    this->x /= vector_length;
    this->y /= vector_length;
  }
}


//------------------------------------------------------------------------non member functions

inline Vector2 Vec2DNormalize(const Vector2 &v)
{
  Vector2 vec = v;

  double vector_length = vec.Length();

  if (vector_length > std::numeric_limits<double>::epsilon())
  {
    vec.x /= vector_length;
    vec.y /= vector_length;
  }

  return vec;
}


inline double Vec2DDistance(const Vector2 &v1, const Vector2 &v2)
{

  double ySeparation = v2.y - v1.y;
  double xSeparation = v2.x - v1.x;

  return sqrt(ySeparation*ySeparation + xSeparation*xSeparation);
}

inline double Vec2DDistanceSq(const Vector2 &v1, const Vector2 &v2)
{

  double ySeparation = v2.y - v1.y;
  double xSeparation = v2.x - v1.x;

  return ySeparation*ySeparation + xSeparation*xSeparation;
}

inline double Vec2DLength(const Vector2& v)
{
  return sqrt(v.x*v.x + v.y*v.y);
}

inline double Vec2DLengthSq(const Vector2& v)
{
  return (v.x*v.x + v.y*v.y);
}


//------------------------------------------------------------------------operator overloads
inline Vector2 operator*(const Vector2 &lhs, double rhs)
{
  Vector2 result(lhs);
  result *= rhs;
  return result;
}

inline Vector2 operator*(double lhs, const Vector2 &rhs)
{
  Vector2 result(rhs);
  result *= lhs;
  return result;
}

//overload the - operator
inline Vector2 operator-(const Vector2 &lhs, const Vector2 &rhs)
{
  Vector2 result(lhs);
  result.x -= rhs.x;
  result.y -= rhs.y;

  return result;
}

//overload the + operator
inline Vector2 operator+(const Vector2 &lhs, const Vector2 &rhs)
{
  Vector2 result(lhs);
  result.x += rhs.x;
  result.y += rhs.y;

  return result;
}

//overload the / operator
inline Vector2 operator/(const Vector2 &lhs, double val)
{
  Vector2 result(lhs);
  result.x /= val;
  result.y /= val;

  return result;
}

///////////////////////////////////////////////////////////////////////////////


//treats a window as a toroid
inline void WrapAround(Vector2 &pos, int MaxX, int MaxY)
{
  if (pos.x > MaxX) {pos.x = 0.0;}

  if (pos.x < 0)    {pos.x = (double)MaxX;}

  if (pos.y < 0)    {pos.y = (double)MaxY;}

  if (pos.y > MaxY) {pos.y = 0.0;}
}

//returns true if the point p is not inside the region defined by top_left
//and bot_rgt
inline bool NotInsideRegion(Vector2 p,
                            Vector2 top_left,
                            Vector2 bot_rgt)
{
  return (p.x < top_left.x) || (p.x > bot_rgt.x) ||
         (p.y < top_left.y) || (p.y > bot_rgt.y);
}

inline bool InsideRegion(Vector2 p,
                         Vector2 top_left,
                         Vector2 bot_rgt)
{
  return !((p.x < top_left.x) || (p.x > bot_rgt.x) ||
         (p.y < top_left.y) || (p.y > bot_rgt.y));
}

inline bool InsideRegion(Vector2 p, int left, int top, int right, int bottom)
{
  return !( (p.x < left) || (p.x > right) || (p.y < top) || (p.y > bottom) );
}

//------------------ isSecondInFOVOfFirst -------------------------------------
//
//  returns true if the target position is in the field of view of the entity
//  positioned at posFirst facing in facingFirst
//-----------------------------------------------------------------------------
inline bool isSecondInFOVOfFirst(Vector2 posFirst,
                                 Vector2 facingFirst,
                                 Vector2 posSecond,
                                 double    fov)
{
  Vector2 toTarget = Vec2DNormalize(posSecond - posFirst);

  return facingFirst.Dot(toTarget) >= cos(fov/2.0);
}

}

#endif //VECTOR2_HPP
