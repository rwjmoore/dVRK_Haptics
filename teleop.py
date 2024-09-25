import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from sensors import footpedal
import sys
import argparse
import csv
pi = 3.1415926
## Important notes:
## For MTM motion:
    # Position: 3D position of MTML (w.r.t to surgeon console) is used to compute the relative translation
    #           applied to PSM1 position (w.r.t to **PSM3**)
    # Orientation: 3D orientation of PSM1 w.r.t PSM3 should always match the orientation of MTML w.r.t 
    #              surgeon console. When user is not driving the PSM (user not present or pressing clutch)
    #              MTML orientation (w.r.t surgeon console) moves to match PSM1 orientation (w.r.t PSM3).
    #              When user is driving PSM1 using MTML (i.e follow mode), PSM3 orientation (w.r.t PSM3)
    #              moves to match MTML orientation (w.r.t surgeon console).
    # Gripper: 1-1 Relationship



# Helper Functions for teleop class

def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

def pickup_transform(offset):

    psm3_T_cam = PyKDL.Frame().Identity()
    psm3_T_cam.p[2] += offset

    return psm3_T_cam

def axis_angle_offset(alignment_offset):

    alignment_offset_angle, _ = alignment_offset.GetRotAngle()
    return alignment_offset_angle * (180/np.pi)



#maps an XYZquat list into a pyKDL frame
def mapXYZquat2KDLframe(XYZquat):
    p = PyKDL.Vector(XYZquat[0], XYZquat[1], XYZquat[2])
    #quat is in the form x,y,z,w
    r = PyKDL.Rotation.Quaternion(XYZquat[3], XYZquat[4], XYZquat[5], XYZquat[6])
    return PyKDL.Frame(r, p)

# class Teleop:



class teleop:
    def __init__(self):
        self.worldcurr_T_probeCommanded = PyKDL.Frame() #frame we send to the HoloLens


    # configuration of the arms
    def configure(self, 
                  parent_arm, 
                  child_arm, 
                  controller_arm, 
                  expected_interval,
                  ignore_operator,
                  scale_factor,
                  initialPose):
        
        print_id('configuring arms for teleoperation')
        
        ## Configuration of arms
        # self.parent_arm = dvrk.psm(arm_name = parent_arm,
        #                            expected_interval = expected_interval)
        
        # self.child_arm = dvrk.psm(arm_name = child_arm,
        #                            expected_interval = expected_interval)
        
        self.controller_arm = dvrk.mtm(arm_name = controller_arm,
                                   expected_interval = expected_interval)

        self.sensors = footpedal()
        ## Select controller (For now keep this blank)
        # self.controller = controller

        self.expected_interval = expected_interval
        self.ignore_operator = ignore_operator
        self.scale_factor = scale_factor
        self.start_teleop = False
        self.worldcurr_T_probeCommanded = PyKDL.Frame() #frame we send to the HoloLens
        self.O_T_teleop = PyKDL.Frame() #snapshot of cam_T_probe that is used to set origin of teleop
        # if len(initialPose) == 7:
        #     print("initialPose Loaded")
        #     self.O_T_teleop = mapXYZquat2KDLframe(initialPose)
        self.hrsv_T_controllerOrigin  = PyKDL.Frame() # variable that holds the absolute pose of the current teleop frame
        self.clutchedPosition = PyKDL.Vector() #variable for storing the clutched position to prevent virtual teleop from moving while clutch is pressed
        print("configured")

    def setting_arm_state(self, arm):

        print_id('starting enable')
        if not arm.enable(10):
            print_id('failed to enable %s' % arm.name())
            sys.exit('failed to enable within 10 seconds')

        print_id('starting home')
        if not arm.home(10):
            print_id('failed to home %s' % arm.name())
            sys.exit('failed to home within 10 seconds')
        # print("arm home position is")
        # print(arm.measured_cp())
        print_id('%s home and enable complete' % arm.name())
        print("arm homed")

    def set_all_arms_state(self):
        print("setting arm state")
        # self.setting_arm_state(self.parent_arm)
        # self.setting_arm_state(self.child_arm)
        self.setting_arm_state(self.controller_arm)

    def run_get(self):
        
        print_id('Read all the relevant transformations.')
        ## Get all relevant transforms to use

        # self.parent_T_cam = pickup_transform(self.cam_offset)
        # self.ecm_T_child = self.child_arm.setpoint_cp()
        # self.ecm_T_parent = self.parent_arm.setpoint_cp()
        self.hrsv_T_controller = self.controller_arm.measured_cp()

    
    #PURPOSE: To align the controller with the child arm
    def align_Controller_Arm(self):
        
        print_id('Aligning controller arm to match pose of child arm with respect to parent arm.')

        ecm_T_cam = self.ecm_T_parent * self.parent_T_cam
        cam_T_ecm = ecm_T_cam.Inverse()

        cam_T_child = cam_T_ecm * self.ecm_T_child
        self.hrsv_T_controller.M = cam_T_child.M

        self.controller_arm.move_cp(self.hrsv_T_controller).wait()

        rospy.sleep(2)

        align_offset_initial = self.controller_arm.measured_cp().M.Inverse() * cam_T_child.M

        self.align_offset = align_offset_initial

        alignment_offset_angle_initial = axis_angle_offset(align_offset_initial)

        print_id('Initial Offset Angle: %s Degrees' % str(round(alignment_offset_angle_initial, 3)))

        return alignment_offset_angle_initial



    #this function returns the initial alignment required to keep the MTM offset from the frame of teleoperation
    def align_Controller_Arm_virtual(self):
        print("aligning controller arms")
        print_id('Aligning controller arm to match pose of intial probe pose with respect to parent arm.')
        self.hrsv_T_controller_home = self.controller_arm.measured_cp()

        self.hrsv_T_controller = PyKDL.Frame()
        ergonomicRotation = PyKDL.Frame()
        yAxis = PyKDL.Vector(0,1,0)
        ergRot = PyKDL.Rotation.Rot(yAxis,-90/180*pi)
        ergonomicRotation.M = ergRot
        self.hrsv_T_controllerOrigin = self.hrsv_T_controller_home*self.O_T_teleop*ergonomicRotation
        print("O_T_teleop arm = ")
        print(self.O_T_teleop)
        self.controller_arm.move_cp(self.hrsv_T_controllerOrigin).wait()
        align_offset_initial = PyKDL.Rotation()
        alignment_offset_angle_initial = axis_angle_offset(align_offset_initial)
        return alignment_offset_angle_initial
    


    def check_teleop_start(self):

        if self.ignore_operator == True:

            while self.start_teleop == False:
                
                # print(self.controller_arm.gripper.measured_js()[0] * (180/np.pi))
                if self.controller_arm.gripper.measured_js()[0] * (180/np.pi) < 60.0:
                    self.start_teleop = True

                rospy.sleep(self.expected_interval)

    
        else:

            while self.start_teleop == False:

                if (self.controller_arm.gripper.measured_js()[0] < 30.0) and (self.sensors.get_operator_event()):
                    self.start_teleop = True
                
                rospy.sleep(self.expected_interval)


    def release_controller(self):

        zero_wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.controller_arm.use_gravity_compensation(True)
        self.controller_arm.body.servo_cf(zero_wrench)

    #PURPOSE: To teleoperate an arm on the da Vinci from the frame of another arm designated as the child arm
    def teleop_child(self):

        hrsv_T_controller_ini = self.controller_arm.measured_cp()
        hrsv_T_controller_origin = self.controller_arm.measured_cp()

        ecm_T_child_ini = self.child_arm.setpoint_cp() ## w.r.t ECM
        ecm_T_cam_ini = self.parent_arm.setpoint_cp()*self.parent_T_cam

        clutch_pressed_prev = False

        while self.start_teleop == True:

            # Check if clutch is pressed or not

            #check rotation between current and origin

            hrsv_T_controller_curr = self.controller_arm.setpoint_cp()

            rotationFrame = hrsv_T_controller_origin.Inverse()*hrsv_T_controller_curr

            rotationEul = rotationFrame.M.GetEulerZYX()
            print(rotationEul)

            # if not any( x > 0.087 for x in rotationEul):
            #     print("smaller")
            #     hrsv_T_controller_curr.M=hrsv_T_controller_origin.M



            ecm_T_parent_curr = self.parent_arm.setpoint_cp() ## w.r.t ECM
                    
            ecm_T_cam_curr = ecm_T_parent_curr * self.parent_T_cam

            ecm_T_child_next = ecm_T_child_ini

            # if self.sensors.flip == False:
            # ecm_T_child_next.M = ecm_T_cam_curr.M * hrsv_T_controller_curr.M * self.align_offset

            #ecm_T_child_next.M = ecm_T_cam_curr.M * hrsv_T_controller_curr.M * self.align_offset * (ecm_T_cam_curr.M.Inverse() * ecm_T_cam_ini.M).Inverse() # Initial from Sayem
            ecm_T_child_next.M = ecm_T_cam_curr.M *hrsv_T_controller_curr.M * self.align_offset

            # else:
            #     ecm_T_child_next.M = hrsv_T_controller_curr.M * self.align_offset
            
            controller_translation = self.scale_factor * (hrsv_T_controller_curr.p - hrsv_T_controller_ini.p)

            # ecm_T_child_next.p = ecm_T_child_next.p + ecm_T_cam_curr.M * controller_translation

            print(clutch_pressed_prev)
            if self.sensors.clutch_pressed == True:
                
                if clutch_pressed_prev == False:
                    self.controller_arm.lock_orientation_as_is()
                    clutch_pressed_prev = True
                
            else:

                ecm_T_child_next.p = ecm_T_child_next.p + ecm_T_cam_curr.M * controller_translation

                if clutch_pressed_prev == True:
                    self.controller_arm.unlock_orientation()
                    clutch_pressed_prev = False
                    
                self.child_arm.move_cp(ecm_T_child_next)

                ecm_T_child_ini = ecm_T_child_next

            rospy.sleep(self.expected_interval)

            hrsv_T_controller_ini = hrsv_T_controller_curr
            ecm_T_cam_ini = ecm_T_cam_curr


    #PURPOSE: The Main Loop for teleoperation. This implementation does not teleoperate a robotic arm but saves
    #         the teleoperated pose to a class variable. This is useful if a teleop loop is run in a separate thread 
    #         and its commanded pose is used for teleoperation of other end effectors (eg: a virtual object)

    #FEATURES: Position control meaning that the pose of the MTM is directly mapped to the commanded pose, which is
    #          worldcurr_T_probeCommanded. Position mapping is relative and according to the controller translation.This
    #          is to facilitate clutch pressing on the console
    #          
    #          The orientation of the MTM is directly mapped to the commanded pose, worldcurr_T_probeCommanded.
    def virtualTeleop(self):
        initialized = self.controller_arm.measured_cp()
        worldcurr_T_probeCommanded_intermediate_ini = self.hrsv_T_controllerOrigin.Inverse() * initialized
    

        clutch_pressed_prev = False
        initialized = False

        while self.start_teleop == True:


            #check rotation between current and origin
            try:

                hrsv_T_controller_curr = self.controller_arm.setpoint_cp()
            except:
                print("unable to get controller arm setpoint")

            # COMPUTE TRANSFORM FROM ORIGIN TO CURRENT POSE
            worldcurr_T_probeCommanded_intermediate = self.hrsv_T_controllerOrigin.Inverse() * hrsv_T_controller_curr

            controller_translation = self.scale_factor * (worldcurr_T_probeCommanded_intermediate.p - worldcurr_T_probeCommanded_intermediate_ini.p)

            # Check if clutch is pressed or not

            if self.sensors.clutch_pressed == True:
                
                if clutch_pressed_prev == False:
                    self.controller_arm.lock_orientation_as_is()
                    clutch_pressed_prev = True
  
                
            else:

                self.worldcurr_T_probeCommanded.p = self.worldcurr_T_probeCommanded.p + controller_translation
                self.worldcurr_T_probeCommanded.M = worldcurr_T_probeCommanded_intermediate.M

                if clutch_pressed_prev == True:
                    self.controller_arm.unlock_orientation()
                    clutch_pressed_prev = False
                    
 

            rospy.sleep(self.expected_interval)
            worldcurr_T_probeCommanded_intermediate_ini = worldcurr_T_probeCommanded_intermediate

   ### ------------------------------------> HAPTIC METHODS <---------------------------------------------

    #PURPOSE: To map the difference in pose between commanded and achieved to a force vector by scaling by scalar k_force
    #ASSUMPTIONS: 
    # The actual probe is placed perpendicular to tissue
    # commandedPose and achievedPose are of the form of a 7 element python list containing [x,y,z,q0,q1,q2,w]
    #NOTE: Useful for basic stiffness controllers
    def mapDistanceToForce(self, commandedPose, achievedPose, k_force):
        if len(commandedPose) != 7 or len(achievedPose) != 7 :
            # print("no recieved pose data")
            return [0.0,0.0,0.0]
        world_T_achieved = mapXYZquat2KDLframe(achievedPose)
        world_T_commanded = mapXYZquat2KDLframe(commandedPose)

        #extract vector that points into tissue 
        z_actual = np.zeros(3)
        z_actual = -1 * pm.toMatrix(world_T_achieved)[0:3, 1] #-y axis points in direction of tissue in unity centroid coordinate frame

        #tracked positions are the marker centroids. Add an offset along the -y axis to both o_desired and o_achieved for more intuitive control
        displacement_to_probe = PyKDL.Frame()
        displacement_to_probe.p = PyKDL.Vector(0.0,-0.105,0.0) #10.5 cm (CHANGE THIS ACCORDING TO TRANSFORM FROM MARKER TO PROBE FRAME)

        #multiply by offset to achieve location of the transducer face
        world_T_achieved = world_T_achieved*displacement_to_probe
        world_T_commanded = world_T_commanded*displacement_to_probe

        #extract position of the two inputted poses 
        o_desired = np.array(pm.toMatrix(world_T_commanded)[0:3,3])
        o_achieved = np.array(pm.toMatrix(world_T_achieved)[0:3,3])

       

        displacement = ( o_desired ) - ( o_achieved )
        
        #compute error projected onto direction of the probe that points into the tissue
        err = np.dot(displacement, z_actual)
        
        force = [0.0,0.0,0.0]
        #if the desired position is below the achieved pose along the z_actual direction
        if err > 0:
            force = err * ( -1*z_actual ) * k_force
        print("displacement= ", displacement)
        print("error = ", err)
        print("force = ", force)

        return force

    
    #PURPOSE: Applys a wrench to the controller arm of the teleoperation. Use for general haptics
    #ASSUMPTIONS: wrench is defined as an np array [Fx,Fy,Fz,t_x,t_y,t_z], where 
    #            F_ is the force, and t_ is the torque in the specified axes. 
    #            The units of F_ *should* be in Newtons and t_ in Newton-meters.
    #NOTE:       Experiments indicate there is not a 1:1 mapping between 
    #            requested force and achieved force. Run hapticProfiling() while holding
    #            the MTM for the mapping. 
    def apply_wrench(self, wrench):
        if len(wrench)!=6:
            print("wrench requires np array of length 6")
            return

        self.controller_arm.body_set_cf_orientation_absolute(True)
        self.controller_arm.body.servo_cf(wrench)
        return
    
    #------------------------------> END OF HAPTIC METHODS <------------------------------

    #--------------------------------> HAPTIC EXAMPLES <---------------------------------------------------

    #PURPOSE: This function applies an increasing force in the z_direction of the console for testing 
    #         the force capabilities of the MTM 
    def applyForce_iterate(self):
        print("Attempting to apply force on MTM")
        
        #Sets haptic feedback w.r.t the console frame. The default is w.r.t pose of the MTM
        self.controller_arm.body_set_cf_orientation_absolute(True)

        appliedWrench = np.array([0.0, 0.0, -2.0, 0.0, 0.0, 0.0])
        z_force = -2.0

        while not rospy.is_shutdown():
            self.controller_arm.body.servo_cf(appliedWrench)
            input("press ay key to continue")
            z_force+=-1.0
            appliedWrench[2]= z_force
            print("applied z force is " + str(z_force))
            rospy.sleep(self.expected_interval)
        
        #apply zero wrench to the controller after test is complete
        appliedWrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.controller_arm.body.servo_cf(appliedWrench)
        return
    
    
    #applies a range of force on the controller arm for characterization of haptic performance
    def hapticProfiling(self):
        print("Profiling force application on MTM")
        self.controller_arm.body_set_cf_orientation_absolute(True)
        appliedWrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        wrenchDict = {"x":0, "y":1, "z":2}
        forceAxis = wrenchDict["z"]
        dataFrame = []
        steps = 500
        effort = np.linspace(-15,15,steps)

        #initialize force application
        appliedWrench[forceAxis] = effort[0]
        self.controller_arm.body.servo_cf(appliedWrench)
        rospy.sleep(5)


        for i in range(0,steps):
            appliedWrench[forceAxis] = effort[i]
            self.controller_arm.body.servo_cf(appliedWrench)
            rospy.sleep(0.05)
            measuredWrench = self.controller_arm.body.measured_cf()
            concatenatedData = np.append(measuredWrench, effort[i])
            print(effort[i])
            print(concatenatedData)
            dataFrame.append(concatenatedData)
            rospy.sleep(self.expected_interval)
        appliedWrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.controller_arm.body.servo_cf(appliedWrench)

        # Specify the CSV file name
        csv_file = 'forceProfile.csv'

        # Save the data to the CSV file
        with open(csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            # Writing the header
            header = ["force_x","force_y","force_z","t_x","t_y","t_z","effort"]
            writer.writerow(header)
            # Writing the data
            writer.writerows(dataFrame)

        print(f'Data saved to {csv_file}')
        return
    
    def forceFeedbackTest(self):
        while not rospy.is_shutdown():
            print("setting arm state")
            self.set_all_arms_state()
            self.run_get()
            print("aligning arm")
            self.align_Controller_Arm()
            print("checking teleop start condition")
            self.check_teleop_start()
            self.release_controller()
            print("applying force")
            self.applyForce_iterate()
            rospy.sleep(self.expected_interval)
            exit()
    
    def hapticProfilingTest(self):
            print("setting arm state")
            self.set_all_arms_state()
            self.run_get()
            print("aligning arm")
            self.align_Controller_Arm()
            print("checking teleop start condition")
            self.check_teleop_start()
            self.release_controller()
            print("applying force")
            self.hapticProfiling()
            rospy.sleep(self.expected_interval)
            exit()


    #PURPOSE: Runs the teleoperation intialization and setup for teleoperation w.r.t another arm
    def run_physical(self):

        while not rospy.is_shutdown():
            
            self.set_all_arms_state()
            self.run_get()
            self.align_Controller_Arm()
            self.check_teleop_start()
            self.release_controller()
            self.teleop_child()

            rospy.sleep(self.expected_interval)
            
    #PURPOSE: Runs the teleoperation initialization and setup for teleoperating a virtual pose
    def run_virtual(self):
        while not rospy.is_shutdown():
            
            self.set_all_arms_state()
            self.run_get()
            self.align_Controller_Arm_virtual()
            self.check_teleop_start()
            self.release_controller()
            self.virtualTeleop()

            rospy.sleep(self.expected_interval)
        

if __name__ == '__main__':
    
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_teleop', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--parent', type=str, default='PSM3',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'parent arm name corresponding to ROS topics without namespace.')

    parser.add_argument('-c', '--child', type=str, default='PSM1',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'child arm name corresponding to ROS topics without namespace.')

    parser.add_argument('-m', '--controller', type=str, default='MTMR',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'controller arm name corresponding to ROS topics without namespace.')
    
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')

    parser.add_argument('-o', '--operator', type=bool, default=False,
                        help = 'Whether headsensor would be considered for teleoperation.')

    parser.add_argument('-s', '--scale', type=float, default=0.2,
                        help = 'Scale factor for teleoperation.')
    
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    initialPose = []#initialize to identity

    application = teleop()
    application.configure(args.parent, 
                          args.child, 
                          args.controller, 
                          args.interval, 
                          args.operator, 
                          args.scale, 
                          initialPose=initialPose) 
    
    application.hapticProfilingTest()
                    
    application.run_virtual() 