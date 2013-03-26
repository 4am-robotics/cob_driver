#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2013 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: 
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: March 2013
#
# \brief
# This module is for characterizing the Care-o-bot 3 battery
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
import pickle

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
    time_values=[]
    
    sg = savitzky.savitzky_golay(window_size=901, order=3)

####################
# Parameters for the Python script
####################

    filename = ''
    robot_name = ''
    try:
        opts, args = getopt.getopt(argv,"hf:r:",["ifile=", "irobot="])
    except getopt.GetoptError:
        print 'bat_char.py -f <filename> -r <robotname>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'bat_char.py -f <filename> -r <robotname>'
            sys.exit()
        elif opt in ("-f", "--ifile"):
            filename = arg
        elif opt in ("-r", "--irobot"):
            robot_name = arg

####################
# Opening the csv file and getting the values for time and voltage
####################
             
    with open(filename, 'rb') as csvfile:

        csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in csvreader:
            row = row[0].split(',')
            volt_v = (float)(row[1])
            if(volt_v <= 48000):
                time_values.append((float)(row[0]))
                volt_values.append((float)(row[1]))
            
####################
# Plotting graphics for the Voltage vs Time
####################

    pylab.figure(1)

    pylab.plot(time_values, volt_values)
    pylab.ylabel("Voltage(mV)")
    pylab.xlabel("Time elapsed(seconds)")
    pylab.title("Volt x Time,file="+ filename.replace('.csv',''))
    pylab.grid(True)
    
    secArray = np.asarray(time_values)
    voltArray = np.asarray(volt_values)
    
    # Polyfit for the voltage values
    z1 = np.polyfit(secArray,voltArray,1)
    z2 = np.polyfit(secArray,voltArray,2)
    z3 = np.polyfit(secArray,voltArray,3)
  
  
    pickle.dump( z1, open( "fits_"+robot_name+".pkl", "a" ) )
    pickle.dump( z2, open( "fits_"+robot_name+".pkl", "a" ) )
    pickle.dump( z3, open( "fits_"+robot_name+".pkl", "a" ) )
    
    # Linear space for better visualization
    xp = np.linspace(0, 12000, 100)
    
    # Generating the polynomial function from the fit
    p1 = np.poly1d(z1)
    p2 = np.poly1d(z2)
    p3 = np.poly1d(z3)
    
    pylab.plot(xp, p1(xp), 'r-', xp, p2(xp), 'g-', xp, p3(xp), 'm-')

    pylab.text(5000, 47000, 'p1=' + p1.__str__(), bbox=dict(facecolor='red', alpha=0.5))
    pylab.text(5000, 46000, 'p2=' + p2.__str__(), bbox=dict(facecolor='green', alpha=0.5))
    pylab.text(5000, 45000, 'p3=' + p3.__str__(), bbox=dict(facecolor='magenta', alpha=0.5))
    
    pylab.savefig(filename.replace('.csv',''), format="pdf")
    #pylab.show()

####################
# Residuals Analysis
####################

    pylab.figure(2)
    
    pylab.ylabel("Residuals(mV)")
    pylab.xlabel("Time elapsed(seconds)")
    pylab.title("Residuals x Time,file="+ filename.replace('.csv',''))
    
    #Evaluating the polynomial through all the time values
    z1_val = np.polyval(z1, time_values)
    z2_val = np.polyval(z2, time_values)
    z3_val = np.polyval(z3, time_values)
    
   # Getting the residuals from the fit functions compared to the real values
    z1_res = volt_values - z1_val
    z2_res = volt_values - z2_val
    z3_res = volt_values - z3_val
    
    pylab.plot(time_values, z1_res, time_values, z2_res, time_values, z3_res)
    
    pylab.grid()
    
    pylab.legend(('Residuals 1st order', 'Residuals 2nd order', 'Residuals 3rd order'))
    
    pylab.savefig(filename.replace('.csv','')+'_res', format="pdf")
    
###################
# Savitzky Golay Filter Applied to the Battery Voltage
###################
    
    values_filt = sg.filter(voltArray)
    
    pylab.figure(3)
    
    pylab.subplot(211)
    
    pylab.plot(time_values, voltArray)
    
    pylab.grid(True)
    pylab.title('Comparison between real and filtered data')
    pylab.ylabel('Real Values(mV)')
    
    pylab.subplot(212)
    
    pylab.plot(time_values, values_filt)
    
    pylab.grid(True)
    pylab.ylabel('Filtered Values(mV)')
    pylab.xlabel('Time(s)')
    
#####    

###################
# Filtered fits
###################
    pylab.figure(4)

    pylab.plot(time_values, values_filt)
    pylab.ylabel("Voltage(mV)")
    pylab.xlabel("Time elapsed(seconds)")
    pylab.title("Volt x Time,file="+ filename.replace('.csv',''))
    pylab.grid(True)
    
    secArray = np.asarray(time_values)
    
    # Polyfit for the voltage values
    z1 = np.polyfit(secArray,values_filt,1)
    z2 = np.polyfit(secArray,values_filt,2)
    z3 = np.polyfit(secArray,values_filt,3)
    
    # Linear space for better visualization
    xp = np.linspace(0, 12000, 100)
    
    # Generating the polynomial function from the fit
    p1 = np.poly1d(z1)
    p2 = np.poly1d(z2)
    p3 = np.poly1d(z3)
    
    pylab.plot(xp, p1(xp), 'r-', xp, p2(xp), 'g-', xp, p3(xp), 'm-')

    pylab.text(5000, 47000, 'p1=' + p1.__str__(), bbox=dict(facecolor='red', alpha=0.5))
    pylab.text(5000, 46000, 'p2=' + p2.__str__(), bbox=dict(facecolor='green', alpha=0.5))
    pylab.text(5000, 45000, 'p3=' + p3.__str__(), bbox=dict(facecolor='magenta', alpha=0.5))

####################
# Residuals Analysis for the filtered Signal
####################

    pylab.figure(5)
    
    pylab.ylabel("Residuals(mV)")
    pylab.xlabel("Time elapsed(seconds)")
    pylab.title("Residuals x Time,file="+ filename.replace('.csv',''))
    
    #Evaluating the polynomial through all the time values
    z1_val = np.polyval(z1, time_values)
    z2_val = np.polyval(z2, time_values)
    z3_val = np.polyval(z3, time_values)
    
   # Getting the residuals from the fit functions compared to the real values
    z1_res = values_filt - z1_val
    z2_res = values_filt - z2_val
    z3_res = values_filt - z3_val
    
    pylab.plot(time_values, z1_res, time_values, z2_res, time_values, z3_res)
    
    pylab.grid()
    
    pylab.legend(('Residuals 1st order', 'Residuals 2nd order', 'Residuals 3rd order'))
    
####################
# Polynomial Evaluation for the filtered signal and the function from the non-moving case
####################

    pylab.figure(6)
    
    poly_vals = np.polyval([-1.55e-10,-5.709e-6,-0.1862,4.825e4], time_values)
    
    pylab.plot(time_values, poly_vals, time_values, values_filt)
      
    pylab.show()
   
if __name__=="__main__":
    main(sys.argv[1:])
