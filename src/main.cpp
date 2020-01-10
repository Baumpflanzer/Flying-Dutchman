#include <Arduino.h>
#include <string>
#include "BoatControl.h"

using namespace std;

string gpxDocument = R"V0G0N(
<?xml version="1.0"?>
<gpx version="1.1" creator="gpxgenerator.com">
<wpt lat="52.443035807275834" lon="13.411784355487839">
    <ele>46.05</ele>
    <time>2020-01-06T16:09:03Z</time>
</wpt>
<wpt lat="52.33785579091809" lon="13.584132377948777">
    <ele>69.81</ele>
    <time>2020-01-06T19:25:56Z</time>
</wpt>
</gpx>
)V0G0N";

void setup() {

  BoatControl flyingDutchman;

  flyingDutchman.addWaypoint(gpxDocument);

  Serial.begin(9600);
  Serial.println("flying Dutchman initialized");
  delay(1000);
  flyingDutchman.goToNextWaypoint();
}

void loop() 
{
}