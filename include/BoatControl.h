#include <Arduino.h>
#include <string>
#include "TinyGPS++.h"

class BoatControl
{
public:
    void addWaypoint(float&, float&);
    void addWaypoint(std::string&);
    int goToNextWaypoint();
private:
    //float tolerance;
    struct m_waypoint{
        float lat;
        float lon;
        bool done;
    };
    std::vector<m_waypoint> getWaypoints()
    {
        return waypoints;
    }
    bool goHere(m_waypoint&, m_waypoint&);
    std::vector<m_waypoint> waypoints;
    m_waypoint getPosition();
    float getOrientation();
    float nextOrientation(m_waypoint&, m_waypoint&, m_waypoint&);
    void drive(float nextOrientation);
};