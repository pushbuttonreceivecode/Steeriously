#ifndef PathFollowingComponent_HPP
#define PathFollowingComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Path.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class PathFollowingComponent
    *\brief An example implementation of the path following steering behavior.
    **/
    class PathFollowingComponent : public steer::Agent
    {
        public:
            PathFollowingComponent(steer::BehaviorParameters* params);
            PathFollowingComponent(steer::Path* p, steer::BehaviorParameters* params);
            virtual ~PathFollowingComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Path Following component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightPathFollowing = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Path Following component.
			**/
			float getWeight()const { return m_weightPathFollowing; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Path Following component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the Path Following component rotation.
			**/
			float getRotation() { return m_rotation; };

			void setPath(steer::Path* p){m_path = p;};
			steer::Path* getPath() const {return m_path;};

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void pathFollowingOn(){m_iFlags |= steer::behaviorType::followPath;};

            bool isPathFollowingOn(){return on(steer::behaviorType::followPath);};
            void pathFollowingOff(){if(on(steer::behaviorType::followPath))   m_iFlags ^=steer::behaviorType::followPath;}

            bool targetAcquired();

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightPathFollowing;///< Multiplier - can be adjusted to effect strength of the Path Following behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
            steer::Path*                m_path;///< pointer to path that the Agent will follow.
            steer::BehaviorParameters*	m_params;///< pointer to flock parameters.
    };
}

#endif // PathFollowingComponent_HPP
