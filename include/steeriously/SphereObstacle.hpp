#ifndef SPHEREOBSTACLE_HPP
#define SPHEREOBSTACLE_HPP

#include <steeriously/Vector2.hpp>

namespace steer
{

/**
 \class SphereObstacle
 \brief Class to aid in obstacle avoidance routine.
**/

class SphereObstacle
{
    public:

        /**
            * \fn SphereObstacle(steer::Vector2 position, float radius);
            * \brief Construct a sphere obstacle from a position and radius.
            * \param position - a steer::Vector2 of floats.
            * \param radius - a plain old float.
        **/
        SphereObstacle(steer::Vector2 position, float radius)
        {
			m_position = position;
			m_radius = radius;
        }

        /// Destructor
        virtual ~SphereObstacle(){};

		/**
		\fn bool taggedInGroup() const;
		\brief Gets the tag status of the obstacle.
		**/
		bool taggedInGroup() const { return m_tag; };

		/**
		\fn void Tag();
		\brief Tags the obstacle for an action.
		**/
		void Tag() { m_tag = true; };

		/**
		\fn void unTag();
		\brief Untags the obstacle.
		**/
		void unTag() { m_tag = false; };

		/**
		\fn void setPosition(steer::Vector2 position);
		\brief Sets the position of the obstacle.
		\param position - a steer::Vector2 of floats.
		**/
		void setPosition(steer::Vector2 position) { m_position = position; };

		/**
		\fn steer::Vector2 getPosition();
		\brief Gets the position of the obstacle.
		**/
		steer::Vector2 getPosition() const { return m_position; };

		/**
		\fn void setRadius(float radius);
		\brief Sets the bounding radius of the obstacle.
		\param radius - a plain old float.
		**/
		void setRadius(float radius) { m_radius = radius; };

		/**
		\fn void getRadius(float radius);
		\brief Gets the bounding radius of the obstacle.
		**/
		float getRadius() const { return m_radius; };

	public:
	    steer::Vector2  m_position;///< Position for the sphere obstacle.
		float           m_radius;///< Radius for the sphere obstacle.
		bool            m_tag;///< Determines wheter or not the obstacle is tagged for an action.
};

}

#endif // SPHEREOBSTACLE_HPP
