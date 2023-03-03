"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import driver, Motor, DistanceSensor
from vehicle import Driver
import math
from controller import DistanceSensor

# create the driver instance.
driver = Driver()

# get the time step of the current world.
timestep = int(driver.getBasicTimeStep())

dsL = DistanceSensor("dsLeft")
dsL.enable(timestep)
dsRR = DistanceSensor("dsRearRight")
dsRR.enable(timestep)
dsRL = DistanceSensor("dsRearLeft")
dsRL.enable(timestep)
dsR = DistanceSensor("dsRight")
dsR.enable(timestep)
dsFL = DistanceSensor("dsFrontLeft")
dsFL.enable(timestep)
dsFR = DistanceSensor("dsFrontRight")
dsFR.enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the driver. Something like:
#  motor = driver.getDevice('motorname')
#  ds = driver.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while driver.step() != -1:
    
    dsRLValue = dsRL.getValue()
    dsLValue = dsL.getValue()
    dsFLValue = dsFL.getValue()
    dsFRValue = dsFR.getValue()
    dsRValue = dsR.getValue()
    dsRRValue = dsFR.getValue()
    
    driver.setCruisingSpeed(10)
    driver.setSteeringAngle(-0.2)
    print("ds:",dsLValue)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
