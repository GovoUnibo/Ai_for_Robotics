#!/usr/bin/env python3

from __future__ import print_function
from environment_pkg.srv import BoxUpdatePos, BoxPickUp, BoxPutDown, DoorOpen, DoorStatus, BoxPos
import rospy

class TutorialClientBox:
    def __init__(self):
        rospy.wait_for_service('BOX1/pick_up')
        self.box1_pick_up = rospy.ServiceProxy('BOX1/pick_up', BoxPickUp)
        rospy.wait_for_service('BOX1/put_down')
        self.box1_put_down = rospy.ServiceProxy('BOX1/put_down', BoxPutDown)
        rospy.wait_for_service('BOX1/update_position')
        self.box1_update_pos = rospy.ServiceProxy('BOX1/update_position', BoxUpdatePos)

        rospy.wait_for_service('BOX2/pick_up')
        self.box2_pick_up = rospy.ServiceProxy('BOX2/pick_up', BoxPickUp)
        rospy.wait_for_service('BOX2/put_down')
        self.box2_put_down = rospy.ServiceProxy('BOX2/put_down', BoxPutDown)
        rospy.wait_for_service('BOX2/update_position')
        self.box2_update_pos = rospy.ServiceProxy('BOX2/update_position', BoxUpdatePos)

        rospy.wait_for_service('BOX3/pick_up')
        self.box3_pick_up = rospy.ServiceProxy('BOX3/pick_up', BoxPickUp)
        rospy.wait_for_service('BOX3/put_down')
        self.box3_put_down = rospy.ServiceProxy('BOX3/put_down', BoxPutDown)
        rospy.wait_for_service('BOX3/update_position')
        self.box3_update_pos = rospy.ServiceProxy('BOX3/update_position', BoxUpdatePos)

        rospy.wait_for_service('BOX1/get_position')
        self.box1_get_pos = rospy.ServiceProxy('BOX1/get_position', BoxPos)

        

    def activate_rdm_positioning_box(self):
        self.box1_update_pos(5, 0.6, True)
        self.box2_update_pos(5, 0.6, True)
        self.box3_update_pos(5, 0.6, True)
    
    def deactivate_rdm_positioning_box(self):
        self.box1_update_pos(0, 0, False)
        self.box2_update_pos(0, 0, False)
        self.box3_update_pos(0, 0, False)

    def pickup_box(self):
        ciaoo = self.box1_pick_up(True)
        # self.box2_pick_up(True)
        # self.box3_pick_up(True)
    
    def putdown_box(self):
        self.box1_put_down(True)
        
        # self.box2_put_down(True)
        # self.box3_put_down(True)

    def get_box_pos(self):
        pos = self.box1_get_pos(True)
        print("Posizione box 1 x: ", pos.x_pos)
        print("Posizione box 1 y: ", pos.y_pos)
        print("Posizione box 1 z: ", pos.z_pos)

class TutorialClientDoor():
    def __init__(self):
        rospy.wait_for_service('Door1/update_pos')
        self.door1_update_pos = rospy.ServiceProxy('Door1/update_pos', DoorOpen)
        rospy.wait_for_service('Door1/status')
        self.door1_status = rospy.ServiceProxy('Door1/status', DoorStatus)

        rospy.wait_for_service('Door2/update_pos')
        self.door2_update_pos = rospy.ServiceProxy('Door2/update_pos', DoorOpen)
        rospy.wait_for_service('Door2/status')
        self.door2_status = rospy.ServiceProxy('Door2/status', DoorStatus)

        rospy.wait_for_service('Door3/update_pos')
        self.door3_update_pos = rospy.ServiceProxy('Door3/update_pos', DoorOpen)
        rospy.wait_for_service('Door3/status')
        self.door3_status = rospy.ServiceProxy('Door3/status', DoorStatus)

        rospy.wait_for_service('Door4/update_pos')
        self.door4_update_pos = rospy.ServiceProxy('Door4/update_pos', DoorOpen)
        rospy.wait_for_service('Door4/status')
        self.door4_status = rospy.ServiceProxy('Door4/status', DoorStatus)

    def open_door(self):
        self.door1_update_pos(True)
        self.door2_update_pos(True)
        self.door3_update_pos(True)
        self.door4_update_pos(True)
    
    def close_door(self):
        self.door1_update_pos(False)
        self.door2_update_pos(False)
        self.door3_update_pos(False)
        self.door4_update_pos(False)

    # def get_door_status(self):
    #     print("Door 1 status: ", self.door1_status())
    #     print("Door 2 status: ", self.door2_status())
    #     print("Door 3 status: ", self.door3_status())
    #     print("Door 4 status: ", self.door4_status())

if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')

    box_client = TutorialClientBox()
    # box_client.pickup_box() 
    box_client.get_box_pos()

    # box_client.activate_rdm_positioning_box()

    # box_client.deactivate_rdm_positioning_box()



    # door_client = TutorialClientDoor()

    # door_client.open_door()
    # import time
    # time.sleep(2)
    # door_client.close_door()

    rospy.spin()

    
