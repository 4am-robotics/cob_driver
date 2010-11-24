#!/usr/bin/python
 

import roslib

roslib.load_manifest('cob_script_server')

import rospy


from simple_script_server import script
 

class InitTray(script):

        def Initialize(self):

               self.sss.init("base")

 
if __name__ == "__main__":

        SCRIPT = InitBase()

        SCRIPT.Start()
