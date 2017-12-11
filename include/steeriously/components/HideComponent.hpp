#ifndef HideComponent_HPP
#define HideComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class HideComponent
    *\brief An example implementation of the hiding steering behavior.
    **/
    class HideComponent : public steer::Agent
    {
        public:
            HideComponent(steer::BehaviorParameters* params);
            virtual ~HideComponent();

            void setParams(steer::BehaviorParameters* params) { m_params = params; };
            steer::BehaviorParameters* getParams() { return m_params; };

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Hide component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightHide = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Hide component.
			**/
			float getWeight()const { return m_weightHide; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Hide component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the
			Hide component rotation.
			**/
			float getRotation() { return m_rotation; };

			void setObstacles(std::vector<SphereObstacle*>* o) { m_obstacles = o; };
            std::vector<SphereObstacle*>* getObstacles() { return m_obstacles; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void HideOn(){m_iFlags |= steer::behaviorType::hide;};

            bool isHideOn(){return on(steer::behaviorType::hide);};
            void HideOff(){if(on(steer::behaviorType::hide))   m_iFlags ^=steer::behaviorType::hide;}

            void setTargetAgent(steer::Agent* a){m_targetAgent = a;};
            steer::Agent* getTargetAgent() const {return m_targetAgent;};

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						                    m_weightHide;///< Multiplier - can be adjusted to effect strength of the Hiding behavior.
            Uint32                                          m_iFlags;///< binary flags to indicate whether or not a behavior should be active
			float						                    m_rotation;///< rotation of the component for applying to drawables or other entities.
            steer::Agent*                                   m_targetAgent;///< The target agent that your entity will be avoiding.
            std::vector<SphereObstacle*>*					m_obstacles;///< pointer to the obstacles needed to avoid them.
            steer::BehaviorParameters*						m_params;///< pointer to flock parameters.
    };
}

#endif // SeekComponent_HPP
