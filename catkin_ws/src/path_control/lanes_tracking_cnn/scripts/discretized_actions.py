#!/usr/bin/env python3

import json
from pynput import keyboard
from std_msgs.msg import String
import rospy

class DiscreteActions():
    # ===================================== INIT==========================================
    def __init__(self):
        self.steerAngle = 20
        self.speed = 0.15
        self.dirKeys   = ['i', 'k', 'j', 'l']
        rospy.init_node('Discreteactionnode', anonymous=False)     
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
        self.keys_publisher = rospy.Publisher('/pathcontrol/keys', String, queue_size=1)

    def run(self):
        """Apply initializing methods and start the threads. 
        """
        with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
            listener.join()

    def keyPress(self,key): 
        try:                                                                        
            if key.char in self.dirKeys:
                command = {}
                if str(key.char) == "i":
                    command['action']        =  '1'
                    command['speed']    =  float(self.speed)
                if str(key.char) == "k":
                    command['action']        =  '3'
                    command['steerAngle']    =  float(0.0)
                if str(key.char) == "j":
                    command['action']        =  '2'
                    command['steerAngle']    =  -float(self.steerAngle)
                if str(key.char) == "l":
                    command['action']        =  '2'
                    command['steerAngle']    =  float(self.steerAngle)
                command = json.dumps(command)
                self.publisher.publish(command)
                self.keys_publisher.publish(str(key.char))
        except: pass

    def keyRelease(self, key):                      #exit key      
        self.publisher.publish('{"action":"2","steerAngle":0.0}')
        if key == keyboard.Key.esc:                        #exit key      
            self.publisher.publish('{"action":"3","steerAngle":0.0}')   
            return False                                      
            
if __name__ == '__main__':
    try:
        nod = DiscreteActions()
        nod.run()
    except rospy.ROSInterruptException:
        pass
