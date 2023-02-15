// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.Math;

public final class Constants {
  public static class OperatorConstants {
    //for Controller
    public static final int DRIVER_CONTROLLER_PORT = 0;
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
    /**
     * 
     * Left motor ID
     * 
     * TLF :Top Left Front Talon
     * TLB :Top Left Back Victor
     * BLF :Bottom Left Front Victor
     * BLB :Bottom Left Back Victor
     */
    public static final int TLF_MOTOR = 1;
    public static final int TLB_MOTOR = 2;
    public static final int BLF_MOTOR = 3;
    public static final int BLB_MOTOR = 4;
    /*
     * Right motor ID
     * TRF : Top Right Front Talon
     * TRB : Top Right Back Victor
     * BRF : Bottom Right Front Victor
     * BRB : Bottom Right Back Victor
     */
    public static final int TRF_MOTOR = 5;
    public static final int TRB_MOTOR = 6;
    public static final int BRF_MOTOR = 7;
    public static final int BRB_MOTOR = 8;
    
    public static final int[] LEFT_ENCODER = new int[]{0,1};
    public static final int[] RIGHT_ENCODER = new int[]{2,3};

    public static final double WHEEL_PERIMETER_CM = 15.24 * Math.PI;
    public static final double DISTENCE_PER_PULS = WHEEL_PERIMETER_CM / 2048;

    public static final double LIMIT_SPEED = 1;
  }

  public static class HandMotorConstants {
    public static final int HAND_PORT = 2;
    public static final double H_OPENSPEED = -0.05;
    public static final double H_CLOSESPEED = 0.05;
    public static final double GEAR_DIAMETER = 5.3;
    public static final double PULSE_PER_REV = 42;
    public static final double CM_PER_PULSE = GEAR_DIAMETER * Math.PI / PULSE_PER_REV;
    public static final double DISTANCE_TO_CLOSE_CUBE_CM = 5;
    public static final double DISTANCE_TO_CLOSE_CONE_CM = 7.5;
  }

  public static class PIDConstants {
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
