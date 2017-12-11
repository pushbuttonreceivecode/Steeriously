#ifndef VECTORMATH_HPP
#define VECTORMATH_HPP

#include <steeriously/SphereObstacle.hpp>
#include <steeriously/Vector2.hpp>

namespace steer //steeriously namespace
{

/**
 \class VectorMath
 \brief Stateless VectorMath class utilizing static methods to manipulate vectors. <br/>Copies of Vector2 are cheap, therefore all functions are "pass by value" at this time.
**/

class VectorMath
{
    public:
        /// Constructor
        VectorMath();
        /// Destructor
        ~VectorMath();

        /// A static method returning a vector that is the component product of two vectors.
        /**
            \sa static Vector2 componentProduct(Vector2 a, Vector2 b)
            \param a - a vector of doubles.
            \param b - a vector of doubles.
        **/
        static Vector2 componentProduct(Vector2 a, Vector2 b);

        /// A static method returning a truncated vector based on some maximum value.
        /**
            \sa static Vector2 truncate(Vector2 v, float max)
            \param a - a vector of doubles.
            \param max - a plain old float.
        **/
        static Vector2 truncate(Vector2 v, float max);

        /// A static method returning the dot product of two vectors.
        /**
            \sa static float dotProduct(Vector2 a, Vector2 b)
            \param a - a vector of doubles.
            \param b - a vector of doubles.
        **/
        static float dotProduct(Vector2 a, Vector2 b);

        /// A static method returning a vector that is perpendicular to the vector supplied.
        /**
            \sa static Vector2 perpendicular(Vector2 a)
            \param a - a vector of doubles.
        **/
        static Vector2 perpendicular(Vector2 a);

        /// A static method returning the directional vector that vector a needs to be transformed to in order to point towards vector b.
        /**
            \sa static Vector2 direction(Vector2 a, Vector2 b)
            \param a - a vector of doubles.
            \param b - a vector of doubles.
        **/
        static Vector2 direction(Vector2 a, Vector2 b);

        /// A static method returning the distance between two given vectors, a and b.
        /**
            \sa static float distance(Vector2 a, Vector2 b)
            \param a - a vector of doubles.
            \param b - a vector of doubles.
        **/
        static float distance(Vector2 a, Vector2 b);

        /// A static method returning the distance squared between two given vectors, a and b - avoids the overhead of sqrt when necessary.
        /**
            \sa static float distanceSquared(Vector2 a, Vector2 b)
            \param a - a vector of doubles.
            \param b - a vector of doubles.
        **/
        static float distanceSquared(Vector2 a, Vector2 b);

        /// A static method returning the angle resolved from a vector (pass in vector1 - vector2 or a precomputed vector) - useful for finding the angle of a target, for example.<br/>Internally, this function converts from radians to degrees so you don't have to.
        /**
            \sa static float findAngle(Vector2 v)
            \param v - a vector of doubles.
        **/
        static float findAngle(Vector2 v);

        /// A static method returning the length (or magnitude) of a vector.
        /**
            \sa static float length(Vector2 v)
            \param v - a vector of doubles.
        **/
        static float length(Vector2 v);

        /// A static method returning the squared length of a vector.
        /**
            \sa static float lengthSquared(Vector2 v)
            \param v - a vector of doubles.
        **/
        static float lengthSquared(Vector2 v);

        /// A static method returning a normalized (or unit) vector.
        /**
            \sa static Vector2 normalize(Vector2 v)
            \param v - a vector of doubles.
        **/
        static Vector2 normalize(Vector2 v);

        /// A static method returning a boolean to test whether a vector intersects a circle.
        /**
            \sa static Vector2 lineIntersectsCircle(Vector2 ahead, Vector2 ahead2, SphereObstacle obstacle)
            \param ahead - a vector of doubles.
            \param ahead2 - a vector of doubles.
            \param obstacle - an steer::SphereObstacle.
        **/
        static bool lineIntersectsCircle(Vector2 ahead, Vector2 ahead2, SphereObstacle obstacle, Vector2 agentPosition);
};

} //steeriously namespace

#endif // VECTORMATH_HPP
