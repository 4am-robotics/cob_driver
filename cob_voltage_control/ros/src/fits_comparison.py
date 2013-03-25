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

import numpy as np

def main(argv):

    volt_values=[]
    time_values=[]
    fits = []
####################
# Parameters for the Python script
####################

    filename = ''
    robot_name = ''
    try:
        opts, args = getopt.getopt(argv,"hf:r:",["ifile=", "irobot="])
    except getopt.GetoptError:
        print 'fits_comparison.py -f <filename> -r <robotname>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'fits_comparison.py -f <filename> -r <robotname>'
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
# Now compare many fits to this function
####################

    with open("fits_"+robot_name+".pkl", 'rb') as fp:
        while(1):
            try:
                fits.append(pickle.load(fp))
            except EOFError:
                break

    for fit in fits:
        z_fit = np.polyval(fit, time_values)
        fit_res = volt_values - z_fit
        pylab.plot(time_values, fit_res)
    
    pylab.savefig("fits_"+ filename.replace('.csv',''), format="pdf")
    pylab.show()

    
if __name__=="__main__":
    main(sys.argv[1:])
