#include <assert.h>

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/Transformations.hpp>

using namespace steer;

steer::Agent::Agent()
	: m_scale(Vector2(1.f, 1.f))
	, m_boundingRadius(0.f)
	, m_tag(false)
	, m_steeringForce(Vector2(0.0, 0.0))
	, m_waypointSeekDistanceSquared(0.f)
	, m_feelers(3)
	, m_velocity(Vector2(0.0, 0.0))
	, m_heading(Vector2(0.0, 0.0))
	, m_side(Vector2(0.0, 0.0))
	, m_mass(1.f)
	, m_timeElapsed(0.f)
	, m_offset(Vector2(0.0, 0.0))
	, m_viewDistance(0.f)
	, m_deceleration(steer::Deceleration::fast)
	, m_boxLength(0.f)
	, m_wallDetectionFeelerLength(0.f)
	, m_maxSpeed(100.f)
	, m_maxForce(400.f)
	, m_maxTurnRate(10.f)
	, m_target(Vector2(0.0, 0.0))
	, m_threatRange(0.f)
	, m_decelerationTweaker(1.f)
	, m_distanceBuffer(0.f)
	, m_wanderTarget(Vector2(0.0, 0.0))
	, m_wanderJitter(0.f)
	, m_wanderRadius(0.f)
	, m_wanderDistance(0.f)
{

}

steer::Agent::Agent(Vector2 position, float radius, Vector2 velocity, Vector2 heading, Vector2 side, float mass, float maxSpeed, float maxForce, float maxTurnRate)
	: m_scale(Vector2(1.f, 1.f))
	, m_boundingRadius(0.f)
	, m_tag(false)
	, m_steeringForce(Vector2(0.0, 0.0))
	, m_waypointSeekDistanceSquared(0.f)
	, m_feelers(3)
	, m_velocity(velocity)
	, m_heading(heading)
	, m_side(side)
	, m_mass(mass)
	, m_timeElapsed(0.f)
	, m_offset(Vector2(0.0, 0.0))
	, m_viewDistance(0.f)
	, m_deceleration(steer::Deceleration::fast)
	, m_boxLength(0.f)
	, m_wallDetectionFeelerLength(0.f)
	, m_maxSpeed(maxSpeed)
	, m_maxForce(maxForce)
	, m_maxTurnRate(maxTurnRate)
	, m_target(Vector2(0.0, 0.0))
	, m_threatRange(0.f)
	, m_decelerationTweaker(1.f)
	, m_distanceBuffer(0.f)
	, m_wanderTarget(Vector2(0.0, 0.0))
	, m_wanderJitter(0.f)
	, m_wanderRadius(0.f)
	, m_wanderDistance(0.f)
{
	setPosition(position);
	setBoundingRadius(radius);
}

steer::Agent::Agent(steer::BehaviorParameters* params)
	: m_scale(params->VehicleScale, params->VehicleScale)
	, m_boundingRadius(params->radius)
	, m_tag(false)
	, m_steeringForce(Vector2(0.0, 0.0))
	, m_waypointSeekDistanceSquared(params->waypointSeekDistance*params->waypointSeekDistance)
	, m_feelers(3)
	, m_velocity(params->velocity)
	, m_heading(params->heading)
	, m_side(params->side)
	, m_mass(params->mass)
	, m_timeElapsed(0.f)
	, m_offset(Vector2(0.0, 0.0))
	, m_viewDistance(params->ViewDistance)
	, m_deceleration(params->deceleration)
	, m_boxLength(params->MinDetectionBoxLength)
	, m_wallDetectionFeelerLength(params->WallDetectionFeelerLength)
	, m_maxSpeed(params->MaxSpeed)
	, m_maxForce(params->MaxForce)
	, m_maxTurnRate(params->MaxTurnRate)
	, m_target(Vector2(0.0, 0.0))
	, m_threatRange(params->ThreatRange)
	, m_decelerationTweaker(params->DecelerationTweaker)
	, m_distanceBuffer(0.f)
	, m_wanderTarget(Vector2(0.0, 0.0))
	, m_wanderJitter(params->wanderJitterPerSecond)
	, m_wanderRadius(params->wanderRadius)
	, m_wanderDistance(params->wanderDistance)
{
	setPosition(params->position);
	setBoundingRadius(params->radius);
}

steer::Agent::~Agent()
{

}

bool steer::Agent::rotateHeadingToFacePosition(Vector2 target)
{
	Vector2 toTarget = VectorMath::normalize(target - getPosition());

	//first determine the angle between the heading vector and the target
	float angle = VectorMath::findAngle(m_heading - toTarget);

	//return true if the agent is facing the target
	if (angle < 0.00001) return true;

	//clamp the amount to turn to the max turn rate
	if (angle > m_maxTurnRate) angle = m_maxTurnRate;

	//finally recreate m_side
	m_side = VectorMath::perpendicular(m_heading);

	return false;
}

void steer::Agent::setHeading(Vector2 newHeading)
{
	assert(VectorMath::lengthSquared(newHeading) - 1.0 < 0.00001);

	m_heading = newHeading;

	//the side vector must always be perpendicular to the heading
	m_side = VectorMath::perpendicular(m_heading);
}

void steer::Agent::createFeelers()
{
	//feeler pointing straight in front
	m_feelers[0] = getPosition() + m_wallDetectionFeelerLength * getHeading();

	//feeler to left
	Vector2 temp = getHeading();
	steer::Vec2DRotateAroundOrigin(temp, HalfPi * 3.5f);
	m_feelers[1] = getPosition() + m_wallDetectionFeelerLength / 2.0f * temp;

	//feeler to right
	temp = getHeading();
	steer::Vec2DRotateAroundOrigin(temp, HalfPi * 0.5f);
	m_feelers[2] = getPosition() + m_wallDetectionFeelerLength / 2.0f * temp;
}

void steer::Agent::setSummingMethod(Uint32 sumMethod)
{

}

bool steer::Agent::accumulateForce(Vector2 &runningTotal, Vector2 forceToAdd)
{
	return true;
}

//---------------------- CalculatePrioritized ----------------------------
//
//  this method calls each active steering behavior in order of priority
//  and acumulates their forces until the max steering force magnitude
//  is reached, at which time the function returns the steering force
//  accumulated to that  point
//------------------------------------------------------------------------
Vector2 steer::Agent::calculatePrioritized()
{
	return Vector2(0.0, 0.0);
}


//---------------------- CalculateWeightedSum ----------------------------
//
//  this simply sums up all the active behaviors X their weights and
//  truncates the result to the max available steering force before
//  returning
//------------------------------------------------------------------------
Vector2 steer::Agent::calculateWeightedSum()
{
	return Vector2(0.0, 0.0);
}

float steer::Agent::getForwardComponent()
{
	return VectorMath::dotProduct(getHeading(), getForce());
}

float steer::Agent::getSideComponent()
{
	return VectorMath::dotProduct(getSide(), getForce());
}

