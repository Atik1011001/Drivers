from Resources.picofunctions import picodriver, Channels, DataTypes
from time import sleep

'''
Channels:
    CHANNEL_1 = 1
    CHANNEL_2 = 2
    CHANNEL_3 = 3
    CHANNEL_4 = 4
    CHANNEL_5 = 5
    CHANNEL_6 = 6
    CHANNEL_7 = 7
    CHANNEL_8 = 8
    MAX_CHANNELS = CHANNEL_8

DataTypes:
    OFF = 0
    PT100 = 1
    PT1000 = 2
    RESISTANCE_TO_375R = 3
    RESISTANCE_TO_10K = 4
    DIFFERENTIAL_TO_115MV = 5
    DIFFERENTIAL_TO_2500MV = 6
    SINGLE_ENDED_TO_115MV = 7
    SINGLE_ENDED_TO_2500MV = 8
    '''

serial_number = 'JR570/085'

'''
Below is an example of creating a function that requests data from the different Channels or calling the functions from the main script.
'''
def get_channel1_data():
    dr = picodriver(serial_number)
    dr.connect()
    dr.set_channel(Channels.CHANNEL_1,DataTypes.PT1000)
    sleep(2)
    # request for data
    value = dr.get_value(Channels.CHANNEL_1)
    print("Tempearture: " + str(value) + "C")
    dr.close()

def get_channel2_data():
    dr = picodriver(serial_number)
    dr.connect()
    dr.set_channel(Channels.CHANNEL_2,DataTypes.PT1000)
    sleep(2)
    # request for data
    value = dr.get_value(Channels.CHANNEL_2)
    print("Tempearture: " + str(value) + "C")
    dr.close()


def main():
    
    # Connect to device
    dr = picodriver(serial_number)
    dr.connect()
    # set device frequency.Note: frequency value has to be 50Hz or 60Hz
    dr.set_frequncey(frequency=50)
    # set device channel
    dr.set_channel(Channels.CHANNEL_1,DataTypes.PT1000)
    dr.set_channel(Channels.CHANNEL_2,DataTypes.PT1000)
    for i in range(10):
        sleep(2)
        # request for data
        value = dr.get_value(Channels.CHANNEL_1)
        print("Tempearture: " + str(value) + "C")

    # close device
    dr.close()
if __name__ == "__main__":
    main()
    