#include <steeriously/Wall.hpp>

steer::Wall::Wall()
: m_renderNormals(false)
, m_vA(Vector2(0.0,0.0))
, m_vB(Vector2(0.0,0.0))
, m_vN(Vector2(0.0,0.0))
{

}

steer::Wall::Wall(bool render, Vector2 A, Vector2 B)
: m_renderNormals(render)
, m_vA(A)
, m_vB(B)
, m_vN(Vector2(0.0,0.0))
{
    CalculateNormal();
}

steer::Wall::Wall(bool render, Vector2 A, Vector2 B, Vector2 N)
: m_renderNormals(render)
, m_vA(A)
, m_vB(B)
, m_vN(N)
{

}

void steer::Wall::CalculateNormal()
{
    Vector2 temp = VectorMath::normalize(m_vB - m_vA);

    m_vN.x = -temp.y;
    m_vN.y = temp.x;
}

void steer::Wall::SetFrom(Vector2 v)

{
    m_vA = v;
    CalculateNormal();
}

void steer::Wall::SetTo(Vector2 v)
{
    m_vB = v;
    CalculateNormal();
}

void steer::Wall::SetNormal(Vector2 n)
{
    m_vN = n;
}
