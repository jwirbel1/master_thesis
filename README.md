This repo contains the code to collect data from the sensors deployed in the harbor.

To run the script on the TI Radar, use the following code:

'python main_plot.py PATH_TO_WRITING_DIR COM-PORT DATA-PORT'

Make sure the path exists, and that the paths to the com and data ports are accessible to the user that executes the script. When running, the console should output first the configuration written to the device, followed by the number of points detected at each frame. When the script is writing data, it also prints the file name to console and updates it once a new file is openend after one minute.
