#Atiku Bugaje
#26/09/2022
#Description: this is the library used to control the power meter.
#             the function available in this library is to get the
#             power measurement and close the resources manager.

import pyvisa

class PM_driver:

    def __init__(self,pmserialnum):
        self.pmserialnum = pmserialnum
        self.rm = pyvisa.ResourceManager()
        self.instr = self.rm.open_resource(self.pmserialnum,)
        
    def get_power_meter(self):
        return self.instr.query("MEASure:POWer?")
        
    
    def close_pm(self):
        #Close the device and the resource manager.
        self.instr.close()
        self.rm.close()