#include <steeriously/components/FlockingComponent.hpp>
#include <steeriously/Steeriously.hpp>
#include <steeriously/BehaviorHelpers.hpp>

using namespace steer;

steer::FlockingComponent::FlockingComponent(steer::BehaviorParameters* params)
	: Agent(params)
	, m_weightSeek(params->SeekWeight)
	, m_weightWander(params->WanderWeight)
	, m_weightAlignment(params->AlignmentWeight)
	, m_weightSeparation(params->SeparationWeight)
	, m_weightCohesion(params->CohesionWeight)
	, m_weightObstacleAvoidance(params->ObstacleAvoidanceWeight)
	, m_weightWallAvoidance(params->WallAvoidanceWeight)
	, m_iFlags()
	, m_rotation(0.f)
	, m_neighbors(nullptr)
	, m_obstacles(nullptr)
	, m_walls(nullptr)
	, m_params(params)
{
    flockingOn();

    //stuff for the wander behavior
	float theta = RandFloat() * TwoPi;

	//create a vector to a target position on the wander circle
	m_wanderTarget = Vector2(m_wanderRadius * cos(theta), m_wanderRadius * sin(theta));
}

steer::FlockingComponent::~FlockingComponent()
{

}

Vector2 steer::FlockingComponent::Calculate()
{
    //reset the steering force
    m_steeringForce = steer::Vector2(0.0, 0.0);

	if(on(steer::behaviorType::alignment) && on(steer::behaviorType::separation) && on(steer::behaviorType::cohesion))
    {
        //tag neighbors...
        TagVehiclesWithinViewRange(this, *m_neighbors, this->m_viewDistance);
    }

    //calculate the force, Luke ;)
	m_steeringForce = calculateWeightedSum();

	return m_steeringForce;
}

Vector2 steer::FlockingComponent::calculateWeightedSum()
{
    if (on(steer::behaviorType::separation))
	{
		m_steeringForce += Separation< FlockingComponent*, std::vector<FlockingComponent*> >(this, *m_neighbors) * m_weightSeparation;
	}

	if (on(steer::behaviorType::alignment))
	{
		m_steeringForce += Alignment< FlockingComponent*, std::vector<FlockingComponent*> >(this, *m_neighbors) * m_weightAlignment;
	}

	if (on(steer::behaviorType::cohesion))
	{
		m_steeringForce += Cohesion< FlockingComponent*, std::vector<FlockingComponent*> >(this, *m_neighbors) * m_weightCohesion;
	}

	if (on(steer::behaviorType::wander))
	{
		m_steeringForce = Wander(this) * m_weightWander;
	}

	if (on(steer::behaviorType::seek))
	{
		m_steeringForce += Seek(this) * m_weightSeek;
	}

	if (on(steer::behaviorType::wallAvoidance))
    {
        m_steeringForce += WallAvoidance< FlockingComponent* >(this, *m_walls) * m_weightWallAvoidance;
    }

    if (on(steer::behaviorType::obstacleAvoidance))
    {
        m_steeringForce += ObstacleAvoidance< FlockingComponent* >(this, *m_obstacles, *m_params) * m_weightObstacleAvoidance;
    }

	m_steeringForce = steer::VectorMath::truncate(m_steeringForce, getMaxForce());

	return m_steeringForce;
}

bool steer::FlockingComponent::targetAcquired()
{
	return getPosition() == getTarget();
}

void steer::FlockingComponent::Update(float dt)
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
