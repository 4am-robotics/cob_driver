#!/usr/bin/python
 

import roslib

roslib.load_manifest('cob_script_server')

import rospy


from simple_script_server import script
 

class InitSdh(script):

        def Initialize(self):

               self.sss.init("sdh")

 
if __name__ == "__main__":

        SCRIPT = InitSdh()

        SCRIPT.Start()
