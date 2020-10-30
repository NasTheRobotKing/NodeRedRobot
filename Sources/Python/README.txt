This python script is located on the raspberryPi at the following location:
/home/pi/PythonProjects/I2C

The scripts communicates with the Arduino Microcontroller through I2C using 
the protocol describes in "I2CProtocolSummary.png"

The code of the microcontroller is found under:
C:\STAR\ROBOTICS\ARDUINO\I2CMotorControl\Sources\Arduino\I2CMotorControl.ino

The microcontroller acts as a slave that receives notifications from the script.
The script is the master that should be called in the Node-Red app

