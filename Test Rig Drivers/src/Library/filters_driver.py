#Atiku Bugaje
#26/09/2022
#Descirption: This is the library that controls the motor filters. 
#             The two filter wheels are controlled from the same library 
#             and functions that is why it has been grouped into a class.
#             the functions available in this library is to set the position of the two filter wheels.

from Resources.python_dll import FWxCListDevices, FWxCOpen, FWxCIsOpen
from Resources.python_dll import SetPosition, FWxCSave



class Motorized_filters:

    def __init__(self,serialnum):
        FWxCListDevices()
        self.serialnum = serialnum
        self.hdl = FWxCOpen(self.serialnum,115200,3)
        FWxCIsOpen(self.serialnum)

    def set_filter_position(self,position):
        SetPosition(self.hdl, position)
        FWxCSave(self.hdl)    
        print("Move to Position: " + str(position)) 


cl_filter = Motorized_filters('TP02792578-24730') #coloured filter
nd_filter = Motorized_filters('TP02691933-23327') #nd Filter

def allmotors(clourfilter,ndfilter):
    cl_filter.set_filter_position(position=clourfilter)
    nd_filter.set_filter_position(position=ndfilter)
       


