# MaxSonar
ROS2 node to publish sensor_msgs/range messages read from the I2CXL-MaxSonar®- EZTM Series High Performance Sonar   Rangefinder MB1202, MB1212, MB1222, MB1232, MB1242
## WARNING
This code compiles, but I still have to wire the maxbotix to the Raspberry Pi and test.(21 Oct 2021)
DONT USE THIS CODE.  It does not work.  It adresses the sonar all wrong.  Look at the code for the GY-US42 sensor.  Change the device address and you 'll have a better chance of success.  (Still trying to procure a I2CXL-MaxSonar in South Africa
