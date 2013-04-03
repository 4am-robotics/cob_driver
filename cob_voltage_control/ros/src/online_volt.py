import csv
import pylab
import sys
import getopt

import savitzky

import numpy as np

from numpy import *
from math import *
from scipy import *

def main(argv):

    volt_values=[]
    time_values = []
    tequation = []
    with open("cob3-3_1.csv", 'rb') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in csvreader:
            row = row[0].split(',')
            volt_v = (float)(row[1])
            if(volt_v <= 48000):
                time_values.append((float)(row[0]))
                volt_values.append((float)(row[1]))


    volt_values_filt = []

    a = 4.446e-08
    b = -0.005834
    c = 257.5
    d = -3.822e+06

    z = np.polyfit(volt_values, time_values, 3)

    v0 = volt_values[0]
    volt_filt = v0*np.ones(123)
    print volt_filt
    sg = savitzky.savitzky_golay(window_size=61, order=3)

    theta = -0.10
    off_y = 5800

    for el in volt_values:
        volt_filt = np.insert(volt_filt, 0, el)
        volt_filt = np.delete(volt_filt, -1)

        vfilt = sg.filter(volt_filt)
        old_settings = np.seterr(all='raise')
        tek = np.polyval([a,b,c,d], vfilt[61])#vfilt[0]*vfilt[0]**a + vfilt[0]**b + c*vfilt[0] + d

        tek = vfilt[61]*sin(theta) + tek*cos(theta)

        tek = tek + off_y

        time_disp = tek

        percent = time_disp/13500
        percent = percent * 100

        if (percent > 100):
            percent = 100.000
        print percent

        volt_values_filt.append(tek)



    pylab.plot(time_values, volt_values_filt)

    pylab.show()



if __name__=="__main__":
    main(sys.argv[1:])
