#ifndef ArriveComponent_HPP
#define ArriveComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class ArriveComponent
    *\brief An example implementation of the arrive steering behavior.
    *       The agent will seek the target with a dampened arrival.
    **/
    class ArriveComponent : public steer::Agent
    {
        public:
            ArriveComponent(steer::BehaviorParameters* params);
            virtual ~ArriveComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Arrive component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightArrive = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Arrive component.
			**/
			float getWeight()const { return m_weightArrive; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Arrive component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the Arrive component rotation.
			**/
			float getRotation() { return m_rotation; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void arriveOn(){m_iFlags |= steer::behaviorType::arrive;};

            bool isArriveOn(){return on(steer::behaviorType::arrive);};
            void arriveOff(){if(on(steer::behaviorType::arrive))   m_iFlags ^=steer::behaviorType::arrive;}

            bool targetAcquired();

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightArrive;///< Multiplier - can be adjusted to effect strength of the arrive behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active.
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
    };
}

#endif // ArriveComponent_HPP
