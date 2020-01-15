/** 
 *  @file   main.cpp
 *  @brief  this is where the main loop is located
 *  @author Maximilian M
 *  @date   2020-01-14
 ***********************************************/

#include <Arduino.h>
#include <string>
#include "BoatControl.h"
#include "GSM.h"

using namespace std;

string gpxDocument = R"V0G0N(
<?xml version="1.0"?>
<gpx version="1.1" creator="gpxgenerator.com">
<wpt lat="52.474765788445794" lon="13.526227732022562">
    <ele>34.72</ele>
    <time>2020-01-15T22:32:45Z</time>
</wpt>
<wpt lat="52.45426710010071" lon="13.56373574288682">
    <ele>32.00</ele>
    <time>2020-01-15T23:13:23Z</time>
</wpt>
</gpx>
)V0G0N";


BoatControl flyingDutchman;

GSM myGSM;

void setup() 
{
  flyingDutchman.addWaypoint(gpxDocument);

  Serial.begin(9600);
  Serial.println("flying Dutchman initialized");
  delay(1000);
}

void loop() 
{
  //myGSM.GSMsetup();
  //myGSM.sendPosition(flyingDutchman.goToNextWaypoint());
  flyingDutchman.goToNextWaypoint();
}
