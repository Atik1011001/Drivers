#Atiku Bugaje
#26/09/2022
#Descirption: This is a demo script of calling the functions to control the devices.           

from Library.filters_driver import Motorized_filters
from Library.filters_driver import allmotors
from Library.pm_driver import PM_driver 
from Library.led_driver import led_driver
from time import sleep


#Devices Serial Num
cl_filter = Motorized_filters('TP02792578-24730') #coloured filter
nd_filter = Motorized_filters('TP02691933-23327') #nd Filter
pm = PM_driver('USB0::0x1313::0x8076::M00869836::INSTR')#powermeter
led = led_driver('USB0::0x1313::0x80C8::M00583544::INSTR')#led

#Test plan
test_plan = {1: {'led': 100, 'colour filter': 5, 'nd filter': 4,},
             2: {'led': 90, 'colour filter': 12, 'nd filter': 3},
             3: {'led': 80, 'colour filter': 3, 'nd filter': 2},
             4: {'led': 70, 'colour filter': 8, 'nd filter': 1},
             5: {'led': 60, 'colour filter': 10, 'nd filter': 5},
             6: {'led': 50, 'colour filter': 7, 'nd filter': 5},
             7: {'led': 40, 'colour filter': 2, 'nd filter': 2},
             8: {'led': 30, 'colour filter': 1, 'nd filter': 1},
             9: {'led': 20, 'colour filter': 9, 'nd filter': 3},
            10: {'led': 10, 'colour filter': 11, 'nd filter':3}}

def main():
    #Open file in the data folder to store the data
    with open('./data/pmdata.txt', 'w', newline='') as f:

        for x, y in test_plan.items():
            print("\nTest:", x)
            data = list(y.values())
            print("*** Starting ***")
            #set filter to test plan
            allmotors(clourfilter=data[1],ndfilter=data[2])
            #set led power to test plan
            led.set_led_power(percentage=data[0])
            #wait till filters and led is ready
            sleep(5)
            #take power mesurement
            pmdata = str(pm.get_power_meter())
            print(pmdata)

            #Write data to txt file in data folder
            f.write("\nTest" + str(x) +" : " + pmdata)
            print("*** End ***")
            
if __name__ == "__main__":
    main()