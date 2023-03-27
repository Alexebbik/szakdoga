import math
from controller import *
from vehicle import *


driver = Driver()
driver.setSteeringAngle(0.5)
#car.setRightSteeringAngle(0)
#car.setLeftSteeringAngle(0)
#driver.setCruisingSpeed(20)
TIME_STEP = 64
ds_list = ["dsRearLeft","dsLeft","dsFrontLeft","dsFrontRight","dsRight","dsRearRight"]
sensor_value = ['','','','','','']
    
    
    
#currentTime = int(driver.getCurrentTime())


'''def __init__(driver):
    wb.wb_driver_init()'''
    
def run():

    
    '''# Initialize the distance sensor
    dsL = DistanceSensor("dsLeft")
    dsL.enable(TIME_STEP)
    dsR = DistanceSensor("dsRight")
    dsR.enable(TIME_STEP)'''
    
    driver.setBrakeIntensity(0.0)
    
    while driver.step() != -1:
        
        '''1for i in range(len(ds_list)):
            sensor_value[i] = ds_list[i].getValue()'''
            
        print("dsRight:",sensor_value[4])
        #sensorR_value = dsRight.getValue()
        #print("dsrightvalue:",sensorR_value)
        '''sensorL_value = dsL.getValue()
        print("dsleft:",sensorL_value)
        sensorR_value = dsR.getValue()
        print("dsright:",sensorR_value)'''
        
        #get the north vector from the compass
        #north = compass.getValues()
        #calculate the orientation of the robot
        #orientation = math.atan2(north[0], north[2])
        #print the orientation
        #print("Orientation:", orientation)
        
        #GPSvalues = gps.getValues()
        #print("GPSvalues:",GPSvalues)
        
        
        #driver.setSteeringAngle(0.5)
        driver.setCruisingSpeed(5)
        #print("currentspeed:",driver.getCurrentSpeed())
        #print("cruisingspeed:",driver.getTargetCruisingSpeed())
        #print("rightangle:",car.getRightSteeringAngle())
        #print("leftangle:",car.getLeftSteeringAngle())
        print("asd")
'''def __del__(driver):
    wb.wb_driver_cleanup()'''
    
if __name__ == "__main__":
    run()
    
    
    
