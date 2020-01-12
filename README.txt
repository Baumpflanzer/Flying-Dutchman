Project to build an autonomous boat. Work in progress.

Core functionalities so far:
- read xml file to extract path and save coordinates in vector
- control boat along path using gps and pre-generated path
  - importance of going back on the path over going to the next waypoint increases exponentially the further away the boat is from the path

Libraries used:
- TinyGPSPlus 1.0.2 by Mikal Hart
- EspSoftwareSerial 6.6.1 by Peter Lerup

Hardware used:
- Esp32
- BN-180 GPS Module
