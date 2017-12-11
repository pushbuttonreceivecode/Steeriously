#include <steeriously/components/WanderComponent.hpp>
#include <steeriously/Steeriously.hpp>

using namespace steer;

steer::WanderComponent::WanderComponent(steer::BehaviorParameters* params)
	: Agent(params)
	, m_weightWander(params->WanderWeight)
	, m_iFlags()
	, m_rotation(0.f)
{
	wanderOn();

	//stuff for the wander behavior
	float theta = RandFloat() * TwoPi;

	//create a vector to a target position on the wander circle
	m_wanderTarget = Vector2(m_wanderRadius * cos(theta), m_wanderRadius * sin(theta));
}

steer::WanderComponent::~WanderComponent()
{

}

Vector2 steer::WanderComponent::Calculate()
{
	//reset the steering force
	m_steeringForce = Vector2(0.f, 0.f);

	if (isWanderOn())
	{
		m_steeringForce = Wander(this) * getWeight();
	}

	return m_steeringForce;
}

void steer::WanderComponent::Update(float dt)
{
	//update the time elapsed
	m_timeElapsed += dt;

	//keep a record of its old position so we can update its cell later
	//in this method
	Vector2 OldPosition = getPosition();

	//calculate the combined force from each steering behavior in the
	//vehicle's list
	Vector2 SteeringForce = Calculate();

	//Acceleration = Force/Mass
	Vector2 acceleration = SteeringForce / getMass();

	//update velocity
	m_velocity += acceleration * dt;

	//make sure vehicle does not exceed maximum velocity
	m_velocity = steer::VectorMath::truncate(m_velocity, getMaxSpeed());

	//update the position
	m_agentPosition += m_velocity * dt;
	m_rotation = steer::VectorMath::findAngle(m_velocity);

	//update the heading if the vehicle has a non zero velocity
	if (steer::VectorMath::lengthSquared(m_velocity) > 0.00000001f)
	{
		m_heading = steer::VectorMath::normalize(m_velocity);

		m_side = steer::VectorMath::perpendicular(m_heading);
	}
}
