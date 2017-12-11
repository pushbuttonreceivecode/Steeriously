#include <steeriously/VectorMath.hpp>

namespace steer
{

/**
* \fn inline bool LineIntersection2D(steer::Vector2 A, steer::Vector2 B, steer::Vector2 C, steer::Vector2 D, float& dist, steer::Vector2& point)
* \brief Checks for an intersection between two lines, given beginning and end points for both lines.
* \param A - a 2D vector of floats.
* \param B - a 2D vector of floats.
* \param C - a 2D vector of floats.
* \param D - a 2D vector of floats.
* \param dist - a plain old float.
* \param point - a 2D vector of floats.
**/
bool LineIntersection2D(steer::Vector2 A, steer::Vector2 B, steer::Vector2 C, steer::Vector2 D, float& dist, steer::Vector2& point);

/**
* \fn void WrapAround(steer::Vector2 &pos, int MaxX, int MaxY);
* \brief Calculates the vector useful for wrapping around the steering vector of an agent to fit in the bounds of a rectangle (the screen, for example).
* \param pos - a 2D vector of floats.
* \param MaxX - a plain old int.
* \param MaxY - a plain old int.
**/
void WrapAround(steer::Vector2 &pos, int MaxX, int MaxY);

}//namespace steer
