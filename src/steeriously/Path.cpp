#include <steeriously/Path.hpp>
#include <steeriously/Utilities.hpp>
#include <steeriously/Transformations.hpp>

using namespace steer;

steer::Path::Path()
: m_looped(false)
, m_currentWaypoint()
, m_numWaypoints(0)
{

}

steer::Path::Path(int NumWaypoints, float MinX, float MinY, float MaxX, float MaxY, bool looped)
: m_looped(looped)
, m_numWaypoints(NumWaypoints)
{
    createRandomPath(NumWaypoints, MinX, MinY, MaxX, MaxY);

    m_currentWaypoint = m_wayPoints.begin();
}

steer::Path::Path(int NumWaypoints, std::list<Vector2>& waypoints)
: m_looped(false)
, m_numWaypoints(0)
, m_wayPoints(waypoints)
{
    m_currentWaypoint = m_wayPoints.begin();
}

steer::Path::~Path()
{

}

void steer::Path::setNextWaypoint()
{
  assert (m_wayPoints.size() > 0);

  if (++m_currentWaypoint == m_wayPoints.end())
  {
    if (m_looped)
    {
      m_currentWaypoint = m_wayPoints.begin();
    }
  }
}

std::list<Vector2> steer::Path::createRandomPath(int NumWaypoints, float MinX, float MinY, float MaxX, float MaxY)
{
    m_wayPoints.clear();

    float midX = (MaxX+MinX)/2.0;
    float midY = (MaxY+MinY)/2.0;

    float smaller = std::min(midX, midY);

    float spacing = TwoPi/(float)NumWaypoints;

    for (int i=0; i<NumWaypoints; ++i)
    {
        float RadialDist = RandInRange(smaller*0.2f, smaller);

        Vector2 temp(RadialDist, 0.0f);

        Vec2DRotateAroundOrigin(temp, i*spacing);

        temp.x += midX;
        temp.y += midY;

        m_wayPoints.push_back(temp);
    }

    m_currentWaypoint = m_wayPoints.begin();

    return m_wayPoints;
}
