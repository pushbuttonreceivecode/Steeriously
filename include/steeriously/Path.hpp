#ifndef PATH_HPP
#define PATH_HPP

#include <list>
#include <vector>
#include <cassert>

#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
        \class Path
        \brief Class providing the necessary structure for path following behavior.
    **/
    class Path
    {
        public:

            /**
            * \fn Path()
            * \brief Default constructor.
            **/
            Path();

            /**
            * \fn Path(int NumWaypoints, std::list<steer::Vector2>& waypoints)
            * \brief Constructor used for creating a path from a list of waypoints.
            * \param NumWaypoints - a plain old int.
            * \param waypoints - a std::list of steer::Vector2.
            **/
            Path(int NumWaypoints, std::list<steer::Vector2>& waypoints);

            /**
            * \fn Path(int NumWaypoints, float MinX, float MinY, float MaxX, float MaxY, bool looped);
            * \brief Constructor for creating a path with initial random waypoints. MinX/Y & MaxX/Y define the bounding box of the path.
            * \param NumWaypoints - a plain old int.
            * \param MinX - a plain old float.
            * \param MinY - a plain old float.
            * \param MaxX - a plain old float.
            * \param MaxY - a plain old float.
            * \param looped - a plain old bool.
            **/
            Path(int NumWaypoints, float MinX, float MinY, float MaxX, float MaxY, bool looped);

            /// Destructor
            ~Path();

            /**
            * \fn steer::Vector2 currentWaypoint() const
            * \brief Returns the current waypoint.
            **/
            steer::Vector2 currentWaypoint() const
            {
                assert(&(*m_currentWaypoint) != NULL);
                return *m_currentWaypoint;
            }

            /**
            * \fn bool finished()
            * \brief Returns true if the end of the list has been reached.
            **/
            bool finished()
            {
                return !(m_currentWaypoint != m_wayPoints.end());
            }

            /**
            * \fn void setNextWaypoint()
            * \brief Moves the iterator on to the next waypoint in the list.
            **/
            void setNextWaypoint();

            /**
            * \fn std::list<steer::Vector2> createRandomPath(int NumWaypoints, float MinX, float MinY, float MaxX, float MaxY)
            * \brief Creates a random path which is bound by rectangle described by the min/max values.
            * \param NumWaypoints - a plain old int.
            * \param MinX - a plain old float.
            * \param MinY - a plain old float.
            * \param MaxX - a plain old float.
            * \param MaxY - a plain old float.
            **/
            std::list<steer::Vector2> createRandomPath(int NumWaypoints, float MinX, float MinY, float MaxX, float MaxY);

            /**
            * \fn void loopOn()
            * \brief Turn on the loop option for the path so the path meets the first waypoint.
            **/
            void loopOn()
            {
                m_looped = true;
            }

            /**
            * \fn void loopOff()
            * \brief Turn off the loop option for the path so the path ends on the last waypoint.
            **/
            void loopOff()
            {
                m_looped = false;
            }

            /**
            * \fn void set(std::list<steer::Vector2> newPath)
            * \brief Method for setting the path with list of vectors.
            * \param newPath - a std::vector of steer::Vector2 floats.
            **/
            void set(std::list<steer::Vector2> newPath)
            {
                m_wayPoints = newPath;

                m_currentWaypoint = m_wayPoints.begin();
            }

            /**
            * \fn  set(const Path& path);
            * \brief Method for setting the path with a previously defined Path.
            * \param path - a steer::Path object.
            **/
            void set(const Path& path)
            {
                m_wayPoints = path.getPath();
                m_currentWaypoint = m_wayPoints.begin();
            }

            /**
            * \fn void clear();
            * \brief Clears all waypoints in the path std::vector.
            **/
            void clear()
            {
                m_wayPoints.clear();
            }

            /**
            * \fn std::list<steer::Vector2> getPath() const;
            * \brief Returns the std::vector of waypoints in the path.
            **/
            std::list<steer::Vector2> getPath() const {return m_wayPoints;}

        private:

            std::list<steer::Vector2>               m_wayPoints;///< Waypoints used to define the path the agent will be steered along.

            //points to the current waypoint
            std::list<steer::Vector2>::iterator     m_currentWaypoint;///< An iterator (pointer) that points to the current waypoint.

            unsigned int                            m_numWaypoints;///< The number of waypoints that define the path.

            bool                                    m_looped;///< Flag to indicate if the path should be looped (The last waypoint connected to the first).
    };
}

#endif // PATH_HPP
