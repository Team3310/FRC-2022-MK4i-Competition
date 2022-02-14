package org.frcteam2910.c2020;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 9;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 0;
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(68.55); //76.3

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 7;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 4;
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(192.33); //254.1

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 14;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 15;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 2;
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(313.95); //249.5

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 1;
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(193.89); //136.5

    public static final int INTAKE_MOTOR_CAN_ID = 2;

    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;

    public static final int PIGEON_PORT = 0;

    
    // Intake
    public static final double INTAKE_COLLECT_RPM = 1550; // 1500
    public static final double INTAKE_SLOW_RPM = 500;
    public static final double INTAKE_RETRACT_RPM = 1000;
    public static final double INTAKE_COLLECT_AUTO_RPM = 1200; // 1500
    public static final double INTAKE_REVERSE_RPM = -2000; // -1500

    public static final int SHOOTER_MASTER_CAN_ID = 13;
    public static final int SHOOTER_SLAVE_CAN_ID = 11;
    public static final int SHOOTER_FEEDER_MOTOR_CAN_ID = 0;

    public static final int HOOD_MOTOR_CAN_ID = 10;
    public static final double HOOD_MIN_ANGLE_DEGREES = 0;
    public static final double HOOD_MAX_ANGLE_DEGREES = 67;
    public static final double HOOD_COMPETITION_HOME_POSITION_DEGREES = 0;

    public static final int ELEVATOR_MOTOR_ID = 3;
    public static final double ELEVATOR_MAX_INCHES = 54.2;
    public static final double ELEVATOR_MIN_INCHES = 0;

    public static final int BALANCE_ELEVATOR_ID = 4;
    public static final double BALANCE_ELEVATOR_MIN_INCHES = 0;
    public static final double BALANCE_ELEVATOR_MAX_INCHES = 0;

    public static final int INDEX_MOTOR_ID = 12;

    public static final int SECONDARY_CONTROLLER_PORT = 1;

    public static final int BALANCE_ELEVATOR_MM_PORT = 0;
    public static final int CLIMB_ELEVATOR_MM_PORT = 1;
    public static final int INDEXER_MM_PORT = 2;
    public static final int HOOD_MM_PORT = 3;

    public static final double MIN_CLIMB_ELEVATOR_PERCENT_BUS = 0.1;

}
