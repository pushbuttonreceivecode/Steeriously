#ifndef GROUPBEHAVIORHELPERS_HPP
#define GROUPBEHAVIORHELPERS_HPP

#include <vector>
#include <steeriously/Vector2.hpp>

namespace steer //namespace steeriously
{

/**
   \fn  template <class T, class conT>
        void steer::TagVehiclesWithinViewRange(const &T entity, const &conT neighbors, float viewDistance);
   \brief Template function that tags any agents within the view range of the specified agent.
   \param entity - a steer::Agent derived object.
   \param neighbors - a std::vector of steer::Agent derived objects.
   \param viewDistance - a plain old float.
**/
template <class T, class conT>
void TagVehiclesWithinViewRange(const T& entity, const conT& neighbors, float viewDistance);

/**
   \fn template <class T, class conT>
       void TagObstaclesWithinViewRange(const T& entity, const conT& obstacles, float boxLength);
   \brief Template function tags any obstacles within the view range of the specified agent.
   \param entity - a steer::Agent derived object.
   \param obstacles - a std::vector of steer::SphereObstacle derived objects.
   \param viewDistance - a plain old float.
**/
template <class T, class conT>
void TagObstaclesWithinViewRange(const T& entity, const conT& obstacles, float boxLength);

/**
    \fn template <class T, class conT>
        void TagNeighbors(const T& entity, conT& neighbors, float radius);
    \brief Template function for tagging neighboring agents.
    \param entity - a steer::Agent derived object.
    \param neighbors - a std::vector of steer::Agent derived objects.
    \param radius - a plain old float.
**/
template <class T, class conT>
void TagNeighbors(const T& entity, const conT& neighbors, float radius);

/**
    \fn template <class T, class conT>
        void TagObstacles(const T& entity, const conT& obstacles, float radius);
    \brief Template function for tagging obstacles.
    \param entity - a steer::Agent derived object.
    \param obstacles - a std::vector of steer::SphereObstacle derived objects.
    \param radius - a plain old float.
**/
template <class T, class conT>
void TagObstacles(const T& entity, const conT& obstacles, float radius);

/**
   \fn inline bool TwoCirclesOverlapped(float x1, float y1, float r1, float x2, float y2, float r2)
   \brief Tests for circle-to-circle overlap.
   \param x1 - a plain old float.
   \param y1 - a plain old float.
   \param r1 - a plain old float.
   \param x2 - a plain old float.
   \param y2 - a plain old float.
   \param r2 - a plain old float.
**/
inline bool TwoCirclesOverlapped(float x1, float y1, float r1, float x2, float y2, float r2);

/**
   \fn inline bool TwoCirclesOverlapped(Vector2 c1, float r1, Vector2 c2, float r2)
   \brief Tests for circle-to-circle overlap.
   \param c1 - a plain old float.
   \param r1 - a plain old float.
   \param c2 - a plain old float.
   \param r2 - a plain old float.
**/
inline bool TwoCirclesOverlapped(Vector2 c1, float r1, Vector2 c2, float r2);

/**
   \fn  template <class T, class conT>
        bool Overlapped(const T* ob, const conT& conOb, float MinDistBetweenObstacles);
   \brief Template function for tagging neighboring agents.
   \param ob - a steer::SphereObstacle derived object.
   \param conOb - a std::vector of steer::SphereObstacle derived objects.
   \param MinDistBetweenObstacles - a plain old float.
**/
template <class T, class conT>
bool Overlapped(const T* ob, const conT& conOb, float MinDistBetweenObstacles);

} //namespace steeriously

#include <steeriously/BehaviorHelpers.inl>

#endif // GROUPBEHAVIORHELPERS_HPP
