// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

public final class Constants {
    public static class ControlConstants {
        public static final int JOYSTICK_PORT = 0;
    }

    public static class DriveTrainConstants {
        /*
         * 
         * Left motor ID
         * 
         * TLF :Top Left Front Talon TLB :Top Left Back Victor BLF :Bottom Left Front Victor BLB :Bottom Left Back Victor
         */
        public static final int TLF_MOTOR = 1;
        public static final int TLB_MOTOR = 2;
        public static final int BLF_MOTOR = 3;
        public static final int BLB_MOTOR = 4;
        /*
         * Right motor ID TRF : Top Right Front Talon TRB : Top Right Back Victor BRF : Bottom Right Front Victor BRB : Bottom Right Back Victor
         */
        public static final int TRF_MOTOR = 5;
        public static final int TRB_MOTOR = 6;
        public static final int BRF_MOTOR = 7;
        public static final int BRB_MOTOR = 8;

        public static final int[] LEFT_ENCODER = new int[] { 0, 1 };
        public static final int[] RIGHT_ENCODER = new int[] { 2, 3 };

        public static final double WHEEL_PERIMETER_CM = 15.24 * Math.PI;
        public static final double DISTANCE_PER_PULS = WHEEL_PERIMETER_CM / 2048;

        public static final double LIMIT_SPEED = 1;
    }

    public static class HandConstants {
        public static final int HAND_PORT = 3;
        public static final int WRIST_PORT = 1;
        public static final double H_OPENSPEED = -0.3;
        public static final double H_CLOSE_CUBE_SPEED = 0.1;
        public static final double H_CLOSE_CONE_SPEED = 1;
        public static final double H_CLOSE_CONE_ENDSPEED = 0.1;
        public static final double H_CLOSE_CUBE_ENDSPEED = 0.05;
        public static final double HOLD_TIME_CONE = 3;
        public static final double HOLD_TIME_CUBE = 1;
        public static final double H_DELAY_CHECK = 0.05;
        public static final double DEADZONE_OFFSET = 0.0005;
    }

    public static class LimeLightConstants {
        public static final double ROTATE_SPEED = 0.4;
        public static final double DRIVE_SPEED = 0.4;
        public static final double ROTATE_MULTIPLIER = 0.4;
    }

    public static class ArmConstants {
        public static final double SETPOINT = 512;
        public static final double KP = 0.15;
        public static final double KI = 0.1;
        public static final double KD = 0.45;
        public static final double DEGREE_PER_PULSE = 360 / 2048;
    }

    public static class PIDConstants {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}
