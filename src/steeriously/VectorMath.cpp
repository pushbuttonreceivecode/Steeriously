#include <math.h>

#include <steeriously/VectorMath.hpp>
#include <steeriously/Utilities.hpp>

using namespace steer;

steer::VectorMath::VectorMath()
{

}

steer::VectorMath::~VectorMath()
{

}

Vector2 steer::VectorMath::componentProduct(Vector2 a, Vector2 b)
{
    return Vector2(a.x*b.x, a.y*b.y);
}

Vector2 steer::VectorMath::truncate(Vector2 v, float max)
{
    float i = max/length(v);
    i = i<1.f ? i : 1.f;
    return v*i;
}

float steer::VectorMath::dotProduct(Vector2 a, Vector2 b)
{
    return (a.x * b.x) + (a.y * b.y);
}

Vector2 steer::VectorMath::perpendicular(Vector2 a)
{
    return Vector2(-a.y, a.x);
}

Vector2 steer::VectorMath::direction(Vector2 a, Vector2 b)
{
    return normalize(Vector2(a - b));
}

float steer::VectorMath::distance(Vector2 a, Vector2 b)
{
    return sqrt(((a.x-b.x)*(a.x-b.x)) + ((a.y-b.y)*(a.y-b.y)));
}

float steer::VectorMath::distanceSquared(Vector2 a, Vector2 b)
{
    float distanceY = a.y - b.y;
    float distanceX = a.x - b.x;

    return distanceY*distanceY + distanceX*distanceX;
}

float steer::VectorMath::findAngle(Vector2 v)
{
    return 180.f / Pi * atan2(v.y,v.x) + 90.f;
}

float steer::VectorMath::length(Vector2 v)
{
    return sqrtf((v.x*v.x) + (v.y*v.y));
}

float steer::VectorMath::lengthSquared(Vector2 v)
{
    return (v.x * v.x + v.y * v.y);
}

Vector2 steer::VectorMath::normalize(Vector2 v)
{
    if(v != Vector2(0.0,0.0))
        v = v/length(v);
    else
        v = Vector2(0.0,0.0);

    return v;
}

bool steer::VectorMath::lineIntersectsCircle(Vector2 ahead, Vector2 ahead2, steer::SphereObstacle obstacle, Vector2 agentPosition)
{
    return distance(obstacle.getPosition(), ahead) <= obstacle.getRadius() || distance(obstacle.getPosition(), ahead2) <= obstacle.getRadius() || distance(obstacle.getPosition(), agentPosition);
}
