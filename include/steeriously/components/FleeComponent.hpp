#ifndef FleeComponent_HPP
#define FleeComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class FleeComponent
    *\brief An example implementation of the flee steering behavior.
    *       The agent will flee from the target when it enters its threat range.
    **/
    class FleeComponent : public steer::Agent
    {
        public:
            FleeComponent(steer::BehaviorParameters* params);
            virtual ~FleeComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Flee component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightFlee = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Flee component.
			**/
			float getWeight()const { return m_weightFlee; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Flee component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the Flee component rotation.
			**/
			float getRotation() { return m_rotation; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void fleeOn(){m_iFlags |= steer::behaviorType::flee;};

            bool isFleeOn(){return on(steer::behaviorType::flee);};
            void fleeOff(){if(on(steer::behaviorType::flee))   m_iFlags ^=steer::behaviorType::flee;}

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightFlee;///< Multiplier - can be adjusted to effect strength of the Fleeing behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
    };
}

#endif // FleeComponent_HPP
