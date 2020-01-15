/** 
 *  @file   BoatControl.cpp
 *  @brief  gets position, calculates the ideal route and controls the boat
 *  @author Maximilian M
 *  @date   2020-01-14
 ***********************************************/

#include <Arduino.h>
#include "SoftwareSerial.h"
#include "BoatControl.h"

using namespace std;
float tolerance = 0.0005;
float aim = 0.001; //0.0001deg = 11m

/**
* add m_waypoint
* @param lat latitude
* @param lon longitude
*/
void BoatControl::addWaypoint(float& lat, float& lon)
{
    m_waypoint wp;
    wp.lat = lat;
    wp.lon = lon;
    wp.done = false;
    waypoints.push_back(wp);
}

/**
* add waypoints
* @param waypoints gpx format
*/
void BoatControl::addWaypoint(string& waypoints)
{
    char latStr[] = "lat=";
    char* endPtr;
    char* latPtr;

    for(size_t found = waypoints.find(latStr, 0); found != string::npos; found = waypoints.find(latStr, found+1)) 
    {
        latPtr = &waypoints[found+5];
        float lat = strtof(latPtr, &endPtr);
        float lon = strtof(endPtr+7, NULL);
        addWaypoint(lat, lon);
    }
}

/**
* gets gps position from gps module
* @return m_waypoint
*/
BoatControl::m_waypoint BoatControl::getPosition()
{
    m_waypoint wp;
    wp.lat = 52.455142;
    wp.lon = 13.526710;
    /*
    TinyGPSPlus gps;
    SoftwareSerial serialConnection(19, 21);
    serialConnection.begin(9600);

    do
    {
        while(serialConnection.available()) //while gps sends characters
        {
            gps.encode(serialConnection.read());
        }
        wp.lat = gps.location.lat();
        wp.lon = gps.location.lng();
    }
    while (gps.location.lat() == 0.0 && gps.location.lng() == 0.0); //until it has read a decent value
    */
    return wp;
}

/**
* gets orientation from compass module
* @return orientation
*/
float BoatControl::getOrientation()
{
    //this will be a sensor reading
    float orientation = 3.141;
    return orientation;
}

/**
* if point is not reached go there
* @param nextWaypoint first m_waypoint that has not been reached yet
* @param lastWaypoint the last m_waypoint that has been reached
* @return angle of the orientation that needs to be reached (from y Axis)
*/
float BoatControl::nextOrientation(m_waypoint& currentPosition, m_waypoint& nextWaypoint, m_waypoint& lastWaypoint)
{
    //make coordinate system with lastWaypoint being at (0, 0)
    float roundEarthFactor = abs(cos(currentPosition.lat)); //making round earth into flat coordinate system
	float nextPtY = nextWaypoint.lat - lastWaypoint.lat;
	float nextPtX = (nextWaypoint.lon - lastWaypoint.lon) * roundEarthFactor;
    float currentPtY = currentPosition.lat - lastWaypoint.lat;
	float currentPtX = (currentPosition.lon - lastWaypoint.lon) * roundEarthFactor;

	//angle between x-Axis and path counterclockwise
	//(path is the vector from lastWaypoint to nextWaypoint)
	float distanceNextPtToLastPt = sqrt(pow(nextPtY, 2) + pow(nextPtX, 2));
	float angle = acos(nextPtX / distanceNextPtToLastPt);
	//determe angle for qudrant III and IV
	if (nextPtY < 0)
	{
		angle = PI * 2 - angle;
	}
	//rotation to get distance between path and currentPosition (on Y axis)
	float distanceToPath = currentPtY * cos(-angle) + currentPtX * sin(-angle);

	float distancePerpendicularPtToLastPt = currentPtX * cos(-angle) - currentPtY * sin(-angle);
	if (distancePerpendicularPtToLastPt > distanceNextPtToLastPt)
	{
		aim = -aim;
	}
	//determine angle needed (from Y axis clockwise)
	float finalAngle = -atan(aim / distanceToPath) + -angle;
	//correct angles
	if (distanceToPath > 0)
	{
		finalAngle = finalAngle + PI;
	}
	//make angle positive
	if (finalAngle < 0)
	{
		finalAngle = finalAngle + 2 * PI;
	}

    return finalAngle;
}

/**
* turns boat to reach nextOrientation, then goes forward 
* @param nextOrientation orientation to be reached
*/
void BoatControl::drive(float nextOrientation)
{
    float currentOrientation = getOrientation();
    //here the motors will be controlled
    //rotate and go into the right direction
}

/**
* if nextWaypoint is not reached go there
* @param nextWaypoint first m_waypoint that has not been reached yet
* @param lastWaypoint the last m_waypoint that has been reached
* @return true if point is reached
*/
bool BoatControl::goHere(m_waypoint& nextWaypoint, m_waypoint& lastWaypoint)
{
    m_waypoint currentPosition;

    for(int i = 0; i<100; i++)
    {
        currentPosition = getPosition();

        float roundEarthFactor = abs(cos(currentPosition.lat));
        //return true if nextWaypoint is reached
        //using pythagorean theorem: if distance between nextWaypoint and lastWaypoint > tolerance
        if (sqrt(pow(currentPosition.lat - nextWaypoint.lat, 2) 
        + pow((currentPosition.lon - nextWaypoint.lon)*roundEarthFactor, 2)) < tolerance)
        {
            nextWaypoint.done = true;
            return true;
        }
        
        float nOrientation = nextOrientation(currentPosition, nextWaypoint, lastWaypoint);
        
        //just for presentation
        Serial.print("finalAngle: ");
        Serial.println(nOrientation*180/PI);
        
        //move in the right direction
        drive(nOrientation);
        delay(3000);
    }
    //point not reached after trying 100 tries (will make something more sophisticated later)
    return false;
}

/**
* Find next m_waypoint and use goHere function
* @return true if point is reached
*/
int BoatControl::goToNextWaypoint()
{
    //find next m_waypoint starting with second m_waypoint
    int i;
    for (i = 1; waypoints[i].done; i++) {}
    if (goHere(waypoints[i], waypoints[i-1]))
    {
        return i;
    }
    else
    {
        return 0;
    }
}