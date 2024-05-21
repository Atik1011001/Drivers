#Atiku Bugaje
#26/09/2022
#descriptions: this is the library used to control the led driver to set the power of the led.
#              the functions available in this library is to set the led power percentage and to close
#              the resource manager

import pyvisa

class led_driver:

    def __init__(self,ledserialnum):
        self.ledserialnum = ledserialnum
        self.rm = pyvisa.ResourceManager()
        self.instr = self.rm.open_resource(self.ledserialnum,)

    def set_led_power(self, percentage):
        #Turning LED Power meter on
        x = (percentage * 5)/ 1000
        self.instr.write("SOURCE1:MODE CC")
        self.instr.write("SOURCE1:CCURENT:CURRENT " + str(x))
        self.instr.write("OUTPUT1:STATE ON")
        print("LED Power: " + str(percentage) + "%")

    def close_led(self):
        #Close the device and the resource manager.
        self.instr.close()
        self.rm.close()