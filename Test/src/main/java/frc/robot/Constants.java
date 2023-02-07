// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.Math;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    //for Controller
    public static final int kDriverControllerPort = 0;
    public static final int JOYSTICK_PORT = 0;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;
    public static final int L_TRIGGER = 2;
    public static final int R_TRIGGER = 3;
    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
  }

  public static class DriveTrainConstants {
    /*
     * 
     * Left motor ID
     * 
     * TLF :Top Left Front Talon
     * TLB :Top Left Back Victor
     * BLF :Bottom Left Front Victor
     * BLB :Bottom Left Back Vivtor
     */
    public static final int TLF_MOTOR = 0;
    public static final int TLB_MOTOR = 1;
    public static final int BLF_MOTOR = 2;
    public static final int BLB_MOTOR = 3;
    /*
     * Right motor ID
     * TRF : Top Right Front Talon
     * TRB : Top Right Back Victor
     * BRF : Bottom Right Front Victor
     * BRB : Bottom Right Back Victor
     */
    public static final int TRF_MOTOR = 4;
    public static final int TRB_MOTOR = 5;
    public static final int BRF_MOTOR = 6;
    public static final int BRB_MOTOR = 7;
    
    public static final int[] LEFT_ENCODER = new int[]{0,1};
    public static final int[] RIGHT_ENCODER = new int[]{2,3};

    public static final double WHEELPERIMETERCM = 15.24 * Math.PI;
    public static final double DISTENCEPERPULS = WHEELPERIMETERCM / 2048;

    public static final double LIMITSPEED = 1;
  }

  public static class HandMotor {
    public static final int HANDPORT = 2;
    public static final double H_OPENSPEED = 0.1;
    public static final double H_CLOSESPEED = -0.1;
    public static final double GEAR_DIAMETER = 5.3;
    public static final double PULSE_PER_REV = 42;
    public static final double CM_PER_PULSE = GEAR_DIAMETER*Math.PI/PULSE_PER_REV;
    public static final double DISTANCE_TO_CLOSE_CUBE_CM = 5;
    public static final double DISTANCE_TO_CLOSE_CONE_CM   = 7.5;
    //mode brake

  }

  public static class PIDConstants {
    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
