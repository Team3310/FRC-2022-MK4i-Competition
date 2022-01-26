package org.frcteam2910.c2020;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 3;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 1;
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(315.0); //76.3

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 0;
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(268.0); //254.1

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 2;
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(239); //249.5

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 3;
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(65); //136.5

    public static final int INTAKE_MOTOR_CAN_ID = 13;

    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;

    public static final int PIGEON_PORT = 8;

    
    // Intake
    public static final double INTAKE_COLLECT_RPM = 1550; // 1500
    public static final double INTAKE_SLOW_RPM = 500;
    public static final double INTAKE_RETRACT_RPM = 1000;
    public static final double INTAKE_COLLECT_AUTO_RPM = 1200; // 1500
    public static final double INTAKE_REVERSE_RPM = -2000; // -1500
}
