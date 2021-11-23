# Toolbox nmea

The nmea ros2 toolbox allows the access to nautical nmea data streams. Therefore, 
this ros2 toolbox is consisting of multiple nodes to support on following used cases

### Use cases
* Receive nmea data sentences from a UDP socket and provide them as a ro2 topic (nmea_sentence)
* Receive nmea data sentences from a text file and provide them as a ros2 topic (nmea_sentence)
* Decode nmea data sentence into individual ros2 topics
* Broadcast nmea data sentences via a UPD socket

