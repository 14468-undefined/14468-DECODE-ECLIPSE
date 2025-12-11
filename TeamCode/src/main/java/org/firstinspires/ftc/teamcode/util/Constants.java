package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public final class Constants{
    @Config
    public static final class DriveConstants{

    }

    @Config
    public static final class FieldConstants{

    }
    @Config
    public static final class intakeConstants{
        public static double INTAKE_POWER = 1;
        public static double REVERSE_INTAKE_POWER = .5;
    }

    @Config
    public static final class shooterConstants{
        public static int FAR_ZONE_SHOT_RPM = 5000;
        public static int MID_SHOT_RPM = 2500;
        public static int CLOSE_SHOT_RPM= 2300;
    }

    @Config
    public static final class transferConstants{
        public static double SPIN_REVERSE_POWER = -1;
        public static double SPIN_POWER = 1;
        public static int CLOSE_TRANSFER_TIME = 500; //in milliseconds
        public static int MID_TRANSFER_TIME = 500; //in milliseconds
        public static int FAR_TRANSFER_TIME = 500; //in milliseconds
    }
    @Config

    public static class Util{
        public static double round(double in, int places){
            return ((int)(in * Math.pow(10,places)))/(double)Math.pow(10,places);
        }
    }

    public static final class WebcamConstants {
        // tag size (meters)

        public static double ROTATION_MAX = 1;
        public static double ROTATION_MIN = 0;
        public static double ROTATION_GROUND = .5;
        public static double ROTATION_GOAL = 1;
        public static double TAG_SIZE = 0.166;

        // Camera intrinsics (your webcam’s calibration)
        public static double FX = 578.272;
        public static double FY = 578.272;
        public static double CX = 402.145;
        public static double CY = 221.506;

        // Desired stopping distance (m)
        public static double DESIRED_DISTANCE = 0.30;

        // PID gains
        public static double kP_FORWARD = 0.5;
        public static double kP_STRAFE = 0.5;
        public static double kP_ROT = 0.5;
    }


}