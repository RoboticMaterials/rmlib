# ~~~ Imports ~~~
# ~~ Standard ~~
import math
import time
import datetime
import os
from math import pi
# ~~ Special ~~
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import pickle

# == Recording ==


def append_states(self, jointArr, forceArr, griprArr, tcpPsArr):
    """ Append joint state and wrist wrench to the appropriate arrays """
    wrench = self.get_wrist_force()
    jointArr.append(self.get_joint_angles())
    forceArr.append(wrench)
    griprArr.append(self.get_gripper_width())
    tcpPsArr.append(self.get_tcp_pose())
    return wrench


def clear_states(jointArr, forceArr, griprArr, tcpPsArr):  # NOTE: Arrays
    """ Clear the arrays of all data """
    jointArr.clear()
    forceArr.clear()
    griprArr.clear()
    tcpPsArr.clear()


def write_and_clear_states(title, jointArr, forceArr, griprArr, tcpPsArr, outPath):  # NOTE: Arrays
    """ Write all the recorded states to a file """
    # FIXME: UNNECESSARY TO OPEN AND CLOSE THE FILE EACH TIME DATA IS ADDED
    # NOTE: This function assumes that 'jointArr' and 'forceArr' have the same number of elements
    dataLen = len(jointArr)
    with open(outPath, 'a') as f:  # Open the file in append text mode
        print("~~~", title, "~~~", file=f)
        for i in range(dataLen):
            line = (jointArr[i], forceArr[i], griprArr[i], tcpPsArr[i])
            print(line, file=f)
        # file closes automatically
    # Clear the arrays of all data
    clear_states(jointArr, forceArr, griprArr, tcpPsArr)

# __ End Record __


# == Assembly Class ==

class Assembly:
    """ Assembly primtives and actions, In general each function should return pass/fail for whether it succeeded """
    # NOTE: These functions assume that the robot is already posed at the approach location
    # NOTE: These functions assume that the gripprt is already oriented in the vertical

    """
    [Y] Assume that the robot is already posed at the approach location - 2019-08-19: Removed references to the starting location
        * Assume that the gripper is at the vertical
    [ ] All pose operations should be with Homogeneous transforms/poses
    """

    def insert_part_tilt_record(self, tilt_angle=5, part_offset=0, dia=0,
                                touch_force=1, insert_force=2, max_movement=0.1,
                                output=False, outPath="insert_part_tilt_record.txt"):
        """ Insert while tilting for circular peg-in-hole , Record during each force condition """
        # NOTE: This function assumes that the offset is from TCP tip (gripper closed) to the bottom of a circular part
        # FIXME: Account for partially closed gripper

        # insert_location : Assume that the robot is already posed at the approach location
        # start_dist : Assume that the robot is already posed at the approach location

        # 1. Create structure for force recording
        jointArr = []  # TODO: Pre-allocate an array for speed
        forceArr = []  # TODO: Pre-allocate an array for speed
        griprArr = []
        tcpPsArr = []

        # 2. Calculate the X offset
        if output:
            print("Part Offset:", (part_offset) *
                  math.sin(math.radians(tilt_angle)))
            print("Dia. Offset:", (dia / 2) *
                  math.cos(math.radians(tilt_angle)))
        x_offset = (part_offset) * math.sin(math.radians(tilt_angle)
                                            ) - (dia / 2) * math.cos(math.radians(tilt_angle))

        # 3. Move by the X offset
        dX = self.translate_tool_relative(xRel=x_offset, yRel=0.0, zRel=0.00)
        self.movel(dX, speed=0.05, frame="tool")
        # 4. Tilt away from the vertical
        dX = self.rotate_tool_relative(
            rxRel=0.0, ryRel=math.radians(tilt_angle), rzRel=0.0)
        self.movel(dX, speed=0.05, frame="tool")

        # 5. Move down till contact
        def a():
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            if wrench[2] < -touch_force:
                return 1
            else:
                return 0

        time.sleep(0.5)
        self.bias_wrist_force()
        plungePose = self.translate_pose(
            self.get_tcp_pose(),
            x=0.0, y=0.0, z=-max_movement,
            frame='base')
        self.movel(plungePose, speed=0.01, stop_condition=a)

        # 5.1. Record move to contact
        if output:
            write_and_clear_states("insert_part_tilt_record::move_contact",
                                   jointArr, forceArr, griprArr, tcpPsArr, outPath)

        def record_only():
            """ Only record the states, not stop """
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            return 0

        # 6. Rotate back to vertical, While moving back by the offset, Record
        dX = self.translate_tool_relative(
            xRel=(x_offset), yRel=0.0, zRel=0.002)  # FIXME: WHY IS Z HARD-CODED?
        dX = self.rotate_pose(
            dX, rx=0.0, ry=-math.radians(tilt_angle), rz=0.0, frame='self')
        self.movel(dX, speed=0.05, frame="tool", stop_condition=record_only)

        if output:
            write_and_clear_states(
                "insert_part_tilt_record::tilt_in", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        # 7.1. Set up conditions for insertion
        def b():
            """ Stop when Z reaction force is too great """
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            if wrench[2] < -insert_force:
                return 1
            else:
                return 0

        def c():
            """ Stop when any force is too great """
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            if min(wrench[0], wrench[1], wrench[2]) < -insert_force:
                return 1
            else:
                return 0

        # 7.2. Complete insertion
        time.sleep(0.5)
        self.bias_wrist_force()

        # 7.3. Move down until contact force exceeds value

        _MOVTOOL = False

        if _MOVTOOL:
            dX = self.translate_tool_relative(
                xRel=0.0, yRel=0.0, zRel=max_movement)
            self.movel(dX, speed=0.01, frame="tool", stop_condition=b)
        else:
            insertPose = self.translate_pose(
                self.get_tcp_pose(), x=0.0, y=0.0, z=-max_movement, frame='base')
            self.movel(insertPose, speed=0.01, frame="base", stop_condition=c)

        if output:
            write_and_clear_states(
                "insert_part_tilt_record::insertion", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        # WARNING: HARD-CODED RETURN
        return True

    def insert_part_spiral_record(self, touch_force=1, drop_force=1, insert_force=2, max_movement=0.1,
                                  lateralStopTorque=0.9,
                                  output=True, outPath="insert_part_spiral_record.txt"):
        """ Insert while spiraling to account for possible misalignment , Record during each force condition """

        if output:
            print("~~~", "insert_part_spiral_record:", "Begin!", "~~~")

        # insert_location : Assume that the robot is already posed at the approach location
        # start_dist : Assume that the robot is already posed at the approach location

        jointArr = []  # TODO: Pre-allocate an array for speed
        forceArr = []  # TODO: Pre-allocate an array for speed
        griprArr = []
        tcpPsArr = []

        def exceeds_Z_force(zLimitF):
            """ Return a function that returns 1 if the z reaction limit is reached """
            def func():
                """ Stops when Z reaction exceeds value """
                wrench = append_states(
                    self, jointArr, forceArr, griprArr, tcpPsArr)
                if wrench[2] < -zLimitF:
                    if output:
                        print("exceeds_Z_force:",
                              "Exceeded the force limit of", -zLimitF)
                    return 1
                else:
                    return 0
            return func

        # 1. Move down till contact
        a = exceeds_Z_force(touch_force)

        time.sleep(0.5)
        self.bias_wrist_force()
        plungePose = self.translate_pose(
            self.get_tcp_pose(), x=0.0, y=0.0, z=-max_movement, frame='base')
        self.movel(plungePose, speed=0.01, stop_condition=a)

        if output:
            write_and_clear_states("insert_part_spiral_record::move_contact",
                                   jointArr, forceArr, griprArr, tcpPsArr, outPath)

        # 2. Spiral

        def b():
            """ Return true if either the later force or the lateral torque has been exceeded """
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            # if abs( wrench[3] ) > .9:
#             if max( abs( wrench[0] ) , abs( wrench[1] ) ) > lateralStopForce:
#                 if output: print( "b:" , "Exceeded the lateral force limit of" , lateralStopForce , "> actual" , abs( wrench[1] ) )
#                 return 1
            if max(abs(wrench[3]), abs(wrench[4])) > lateralStopTorque:
                if output:
                    print("b:", "Exceeded the lateral torque limit of", lateralStopTorque,
                          "> (one of) actual", abs(wrench[3]), abs(wrench[4]))
                return 1
            elif wrench[2] > -drop_force:
                if output:
                    print("b:", "Drop force",
                          wrench[2], "> actual", -drop_force)
                return 1
            else:
                return 0

        # FIXME: SHOULD THIS BE SCALED TO THE PART SIZE?
        self.spiral(15, 0.0005, stepSize=0.00005, maxRadius=0.004,
                    speed=0.002, accel=0.5, stop_condition=b)

        if output:
            write_and_clear_states(
                "insert_part_spiral_record::spiral", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        if insert_force > 0.0:
            if output:
                print("Monitoring final insert force ...")
            a = exceeds_Z_force(insert_force)
            time.sleep(0.5)
            self.bias_wrist_force()
            plungePose = self.translate_pose(
                self.get_tcp_pose(), x=0.0, y=0.0, z=-max_movement, frame='base')
            self.movel(plungePose, speed=0.01, stop_condition=a)

        if output:
            write_and_clear_states(
                "insert_part_spiral_record::insert", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        if output:
            print("\n~~~", "insert_part_spiral_record:", "End!", "~~~\n\n")

        # WARNING: HARD-CODED RETURN
        return True

    def limited_z_push_record(self, poseAbove, zForceLimit, depthLimit,
                              output=False, outPath="limited_z_push_record.txt"):
        """ Starting from 'poseAbove', push down in Z until either 'zForceLimit' or 'depthLimit' is reached """

        jointArr = []  # TODO: Pre-allocate an array for speed
        forceArr = []  # TODO: Pre-allocate an array for speed
        griprArr = []
        tcpPsArr = []

        self.movej(poseAbove)

        # Move down till contact
        def a():
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            if wrench[2] < -zForceLimit:
                return 1
            else:
                return 0
        time.sleep(0.5)
        self.bias_wrist_force()

        self.movel(np.add(self.get_tcp_pose(), [
                   0.0, 0.0, -depthLimit, 0.0, 0.0, 0.0]), speed=0.01, stop_condition=a)
        if output:
            write_and_clear_states(
                "limited_z_push_record::tamp", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        self.movel(np.add(self.get_tcp_pose(), [
                   0.0, 0.0,  depthLimit, 0.0, 0.0, 0.0]), speed=0.01)

        # WARNING: HARD-CODED RETURN
        return True

    def grip_and_twist(self, pose, angleSweep, zForceLimit, depthLimit, numSweeps,
                       output=False, outPath="grip_and_twist.txt"):
        """ Move to pose, Close gripper, twist, Release gripper """

        jointArr = []  # TODO: Pre-allocate an array for speed
        forceArr = []  # TODO: Pre-allocate an array for speed
        griprArr = []
        tcpPsArr = []

        self.open_gripper()
        self.movel(pose)
        self.close_gripper()

        # Move down till presure
        def a():
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            if wrench[2] < -zForceLimit:
                return 1
            else:
                return 0

        time.sleep(0.5)
        self.bias_wrist_force()

        self.movel(np.add(self.get_tcp_pose(), [
                   0.0, 0.0, -depthLimit, 0.0, 0.0, 0.0]), speed=0.01, stop_condition=a)
        if output:
            write_and_clear_states(
                "grip_and_twist::push", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        origAngles = self.get_joint_angles()
        hiWristQ = origAngles[:]
        hiWristQ[5] += angleSweep
        loWristQ = origAngles[:]
        loWristQ[5] -= angleSweep

        def record_only():
            """ Only record the states, not stop """
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            return 0

        for i in range(numSweeps):
            self.set_joint_angles(hiWristQ, stop_condition=record_only)
            self.set_joint_angles(loWristQ, stop_condition=record_only)
        self.set_joint_angles(origAngles, stop_condition=record_only)
        if output:
            write_and_clear_states(
                "grip_and_twist::twists", jointArr, forceArr, griprArr, tcpPsArr, outPath)

        self.open_gripper()

        # WARNING: HARD-CODED RETURN
        return True

    def thread_nut_record(self, studPose, aboveDist, advByFlats, zForceLimit, zTorqLim, pitch_mm,
                          output=False, outPath="thread_nut_record.txt"):
        """ Attempt to thread a nut onto a stud, record each twist operation """
        # 2019-04-19: Start simple, assume no cross-threading
        #    * Stop condition should be the same for success and cross-threading, hi z torque
        #    * How to measure success? - Number of successful turns? - Out of scope for this function

        #  0. Allocate recording vars
        jointArr = []
        forceArr = []
        griprArr = []
        tcpPsArr = []

        #  1. Calc the per-turn angle , Turns should be in integer increments of flats * 60 deg
        perFlatAng = pi/3.0
        flatLims = [1, 3]  # Limits on flat advancement [ minFlats , maxFlats ]
        incrFlats = min(flatLims[1], max(flatLims[0], int(advByFlats)))
        perTurnAng = incrFlats * perFlatAng

        #  2. Calculate the above pose
        abovePose = studPose[:]
        abovePose[2] += aboveDist

        #  3. Descend on the stud with a z force limit
        plungeDist = aboveDist * 1.2
        # Move down till contact

        def b():
            wrench = self.get_wrist_force()
            if wrench[2] < -zForceLimit:
                return 1
            else:
                return 0

        time.sleep(0.5)
        self.bias_wrist_force()
        self.movel(np.add(self.get_tcp_pose(), [
                   0.0, 0.0, -plungeDist, 0.0, 0.0, 0.0]), speed=0.01, stop_condition=b)

        #  4. Calc twist angles
        origAngles = self.get_joint_angles()
        hiWristQ = origAngles[:]
        hiWristQ[5] += perTurnAng/2.0
        loWristQ = origAngles[:]
        loWristQ[5] -= perTurnAng/2.0

        #  5. Init to low angle and calc poses
        self.set_joint_angles(loWristQ)
        loWristP = self.get_tcp_pose()
        rotatedXform = self.rotate_transform(
            self.convert_pose_to_transform(self.get_tcp_pose()), rz=perTurnAng)
        hiWristP = self.convert_transform_to_pose(rotatedXform)
        # Distance to decend per turn [m]
        plungePerTurn = (pitch_mm / 1000.0) * (incrFlats / 6.0)

        def bump_poses_down():
            # FIXME: THIS IS NOT GREAT, Turning and then moving down created tension on the arm and causes it to release with a "snap"
            #        This sometimes causes some rotation of the nut
            loWristP[2] -= plungePerTurn
            hiWristP[2] -= plungePerTurn

        #  6. Tight/cross condition
        def a():
            a.cond = False
            wrench = append_states(
                self, jointArr, forceArr, griprArr, tcpPsArr)
            if abs(wrench[5]) > zTorqLim:
                a.cond = True
                return 1
            else:
                return 0
        a.cond = False

        #  7. For each twist
        i = 1
        nutLoose = True
        turndOnce = False

        time.sleep(0.5)
        self.bias_wrist_force()
        while(nutLoose):
            #  8. Twist to the high angle
            if 0:
                self.set_joint_angles(
                    hiWristQ, speed=0.75, accel=1.05, stop_condition=a)
            else:
                self.movej(hiWristP, speed=0.75, accel=1.05, stop_condition=a)
            i += 1
            #  9. If the condition was met, break
            if a.cond:
                break
            # 10. Ungrip
            self.set_gripper_width(0.030)
            # 11. Move to the low angle  &&  Plunge to new depth
            bump_poses_down()
            self.movej(loWristP)
            # 12. Grip
            self.close_gripper()
        if output:
            write_and_clear_states(
                "thread_nut_record::" + str(i), jointArr, forceArr, griprArr, tcpPsArr, outPath)

        # WARNING: HARD-CODED RETURN
        return True

# __ End Asm __
