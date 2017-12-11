#include <iostream>

#include <steeriously/Agent.hpp>
#include <steeriously/VectorMath.hpp>
#include <steeriously/Transformations.hpp>

template <class T, class conT>
void steer::TagVehiclesWithinViewRange(const T& entity, const conT& neighbors, float viewDistance)
{
	steer::TagNeighbors<T, conT>(entity, neighbors, viewDistance);
}

template <class T, class conT>
void steer::TagObstaclesWithinViewRange(const T& entity, const conT& obstacles, float boxLength)
{
	steer::TagObstacles<T, conT>(entity, obstacles, boxLength);
}

template <class T, class conT>
void steer::TagNeighbors(const T& entity, const conT& neighbors, float radius)
{
	//iterate through all entities checking for range
	for (auto& i : neighbors)
	{
		if (entity != nullptr && i != nullptr && i != entity)
		{
			//first clear any current tag
			i->unTag();

			Vector2 to = i->getPosition() - entity->getPosition();

			//the bounding radius of the other is taken into account by adding it
			//to the range
			float range = radius + i->getBoundingRadius();

			//if entity within range, tag for further consideration. (working in
			//distance-squared space to avoid sqrts)
			if (steer::VectorMath::lengthSquared(to) < range*range)
			{
				i->Tag();
			}
		}

	}//next entity
}

template <class T, class conT>
void steer::TagObstacles(const T& entity, const conT& obstacles, float radius)
{
	//iterate through all entities checking for range
	for (auto& i : obstacles)
	{
		if (entity != nullptr && i != nullptr)
		{
			//first clear any current tag
			i->unTag();

			Vector2 to = i->getPosition() - entity->getPosition();

			//the bounding radius of the other is taken into account by adding it
			//to the range
			float range = radius + i->getRadius();

			//if entity within range, tag for further consideration. (working in
			//distance-squared space to avoid sqrts)
			if (steer::VectorMath::lengthSquared(to) < range*range)
			{
				i->Tag();
			}
		}

	}//next entity
}

bool steer::TwoCirclesOverlapped(float x1, float y1, float r1, float x2, float y2, float r2)
{
	float DistBetweenCenters = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

	if ((DistBetweenCenters < (r1 + r2)) || (DistBetweenCenters < fabs(r1 - r2)))
	{
		return true;
	}

	return false;
}

bool steer::TwoCirclesOverlapped(Vector2 c1, float r1, Vector2 c2, float r2)
{
	float DistBetweenCenters = sqrt((c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y));

	if ((DistBetweenCenters < (r1 + r2)) || (DistBetweenCenters < fabs(r1 - r2)))
	{
		return true;
	}

	return false;
}

template <class T, class conT>
bool steer::Overlapped(const T* ob, const conT& conOb, float MinDistBetweenObstacles)
{
	typename conT::const_iterator it;

	for (it = conOb.begin(); it != conOb.end(); ++it)
	{
		if (TwoCirclesOverlapped(ob->getPosition(), ob->getRadius() + MinDistBetweenObstacles, it->getPosition(), it->getRadius()))
		{
			return true;
		}
	}

	return false;
}
