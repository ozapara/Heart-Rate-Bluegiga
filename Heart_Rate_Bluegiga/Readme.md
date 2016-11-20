# Heart-Rate-Bluegiga
Application connects to heart rate device using Bluegiga dongle and parses heart rate parameters, heart rate measurment, energy, rr intervals

Application based on the thermometer-demo example in Bluegiga SDK
It uses only "C" BGLib/BGApi Bluegiga SDK
Heart Rate Application is cross-platform. (Tested on Windows with Visual Studio 2015)


# How to use application

1. Install Bluegiga Drivers from official site
2. Install Bluegiga SDK from official site
3. Compile Heart Rate Application
4. To scan bluetooth device start program with "<COMx|list> scan" parameters
5. To connect to heart rate device, use "<COMx|list> <scan|address [addess_type]>"
6. To get info use "<COMx|list> info"
addess_type is 0 by default, but some devices have 1

Examples:

scan:
C:\Heart_Rate_Bluegiga\Debug>Heart_Rate_Bluegiga.exe COM3 scan
6b:e9:2a:5b:02:5c RSSI:4294967257 Name:Unknown addess_type:1
60:03:08:a3:2e:ff RSSI:4294967257 Name:Unknown addess_type:0

connect:
C:\Heart_Rate_Bluegiga\Debug>Heart_Rate_Bluegiga.exe COM3 f2:ea:14:de:9c:35 1
Trying to connect
DEBUG: State changed: disconnected --> connecting
DEBUG: State changed: connecting --> connected
Connected
DEBUG: State changed: connected --> finding_services
DEBUG: State changed: finding_services --> finding_attributes
heart_rate_handle_measurement was found
heart_rate_handle_configuration was found
DEBUG: State changed: finding_attributes --> listening_measurements
Heart rate measurment value: 82  Rr intervals: 
Heart rate measurment value: 82  Rr intervals: 731
Heart rate measurment value: 82  Rr intervals: 731 731


There is useful video Getting started with Bluegiga's DKBLE112
https://www.youtube.com/watch?v=RH5Q1fvc_Do