import time
import numpy as np
import serial
import serial.serialutil
import json,os
from Radarfunctions import Radarfunctions
import datetime as datetime
import sys
import gzip

uploadData=False
verbose = False
writeCSV = True
f = None

# get the folder name from the command line
folder = sys.argv[1]
CLI = sys.argv[2]
DATA = sys.argv[3]

start_time = time.time()

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

RadFunc = Radarfunctions(configFileName="config_long_range_2.cfg", 
                          usbportCLI=CLI, usbportDATA=DATA, baudrateCLI=115200, baudrateData=921600,
                          bytezise=serial.EIGHTBITS,parity=serial.PARITY_NONE,timeout=1,
                          byteBufferLength=0,byteBuffer=np.zeros(2**15,dtype = 'uint8'))

CLIport,Dataport = RadFunc.serialConfig()

try:
    print("Starting sensor")
    CLIport.write(('sensorStart\n').encode())
    #print(CLIport.readline())
except Exception as e:
    print("error starting sensor")
    print(e)
    exit()

last_call_time = 0
batch_no = 0
operations = []
targets = []
points = []
infos = []
heatMap = {}

h = 0   
s = 0

writeData = []
while True: 
    error = 0   
    try:
        writeTime = datetime.datetime.now().time()
        # convert from utc to european winter time
        # set writeCSV to True only between 12:00 and 13:00, False otherwise
        if writeTime >= datetime.time(12, 0) and writeTime <= datetime.time(13, 0):
            writeCSV = True
        else:
            writeCSV = False

        dataOk,targets,points,infos,heatMap = RadFunc.update(verbose)
        data = {"timestamp": None, "points": [],"heatMap": None}
        data["timestamp"] = time.time()
        data["heatMap"] = heatMap
        for i in range(len(points)):
            data["points"].append([points[i]['x'], points[i]['y'], points[i]['z'], points[i]['velocity_x'], 
                                   points[i]['velocity_y'], points[i]['velocity_z'], points[i]['range'], points[i]['doppler'],
                                   points[i]['azimuth'], points[i]['elevation']])

        # add the data to the writeData object if data is not empty
        if len(data["points"]) > 0 and len(data["heatMap"]) > 0 and writeCSV == True:
            writeData.append(data)      

        current_time = time.time()
        elapsed_time = current_time - last_call_time

        if f is not None:
            since_fileopen = current_time - fileopen_time

        # write the data to a csv file
        if writeCSV:
            if f is None:  
                print("opening file")  
                # Get the current date  
                current_date = datetime.date.today().strftime("%Y-%m-%d")  
                # Check if the folder 'mydate' exists  
                mydate_folder = folder + "/" + current_date  
                if not os.path.exists(mydate_folder):  
                    os.makedirs(mydate_folder)  
                # Create the filename using the current date and time  
                filename = f"{mydate_folder}/{str(current_time)}.jsonl.gz"  # Change the extension to .jsonl.gz  
                print("opening file: " + filename)  
                f = gzip.open(filename, "at")  # Open the file with gzip in append mode and text mode  
                fileopen_time = time.time()  
            elif since_fileopen > 60:  
                current_date = datetime.date.today().strftime("%Y-%m-%d")  
                # Check if the folder 'mydate' exists  
                mydate_folder = folder + "/" + current_date  
                if not os.path.exists(mydate_folder):  
                    os.makedirs(mydate_folder)  
                # Create the filename using the current date and time  
                filename = f"{mydate_folder}/{str(current_time)}.jsonl.gz"  # Change the extension to .jsonl.gz  
                print("opening file: " + filename)  
                f.close()  # Close the previous file before opening a new one  
                f = gzip.open(filename, "at")  # Open the file with gzip in append mode and text mode  
                fileopen_time = time.time()  
            # Convert to a json string and write to the file  
            if len(writeData) > 0:  
                testjs = json.dumps(writeData, cls=NumpyEncoder)  
                f.write(testjs + "\n")  # Write the JSON string with a newline  
                f.flush()  # Flush the buffer to ensure data is written immediately  
                writeData = []  

    except Exception as e:
        print("error: " + str(e), "error:", error)
        continue

    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        print("Sending sensorStop command")
        CLIport.close()
        Dataport.close()
        exit()   