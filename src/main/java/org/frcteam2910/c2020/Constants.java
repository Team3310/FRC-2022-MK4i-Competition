package org.frcteam2910.c2020;

public class Constants {
    public static final double DRIVETRAIN_VOLTAGE_RAMP = 5.0;


    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 9;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 0;

    //public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(68.81); //68.81 Practice robot settings
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(64.07); //Comp settings

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 7;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 4;

    //public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(195.11); //192.33 Practice robot settings
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(152.49); //Comp settings

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 14;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 15;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 2;
    //public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(314.73); //314.73 Practice robot settings
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(48.34); //Comp settings

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 1;
    
    //public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(192.91); //192.91 Practice robot settings
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(213.48); //Comp settings

    public static final int INTAKE_MOTOR_CAN_ID = 2;

    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    public static final int PRIMARY_CONTROLLER_PORT = 0;

    public static final int PIGEON_PORT = 0;

    
    // Intake
    public static final double INTAKE_COLLECT_RPM = 1550; // 1500
    public static final double INTAKE_SLOW_RPM = 500;
    public static final double INTAKE_RETRACT_RPM = 1000;
    public static final double INTAKE_COLLECT_AUTO_RPM = 1200; // 1500
    public static final double INTAKE_REVERSE_RPM = -1000; // -1500

    public static final int SHOOTER_MASTER_CAN_ID = 13;
    public static final int SHOOTER_SLAVE_CAN_ID = 11;
    public static final int SHOOTER_FEEDER_MOTOR_CAN_ID = 0;

    public static final int HOOD_MOTOR_CAN_ID = 10;
    public static final double HOOD_MIN_ANGLE_DEGREES = 0;
    public static final double HOOD_MAX_ANGLE_DEGREES = 50;
    public static final double HOOD_COMPETITION_HOME_POSITION_DEGREES = 0;

    public static final int ELEVATOR_MOTOR_ID = 3;
    public static final double ELEVATOR_MAX_INCHES = 54.8;
    public static final double ELEVATOR_STAGE_ONE_INCHES = 35;
    public static final double ELEVATOR_STAGE_TWO_INCHES = 5;
    public static final double ELEVATOR_MIN_INCHES = -2.0;

    public static final int BALANCE_ELEVATOR_ID = 4;
    public static final double BALANCE_ELEVATOR_MIN_INCHES = 0;
    public static final double BALANCE_ELEVATOR_MAX_INCHES = 18.9;

    public static final int INDEX_MOTOR_ID = 12;

    public static final int SECONDARY_CONTROLLER_PORT = 1;

    public static final int BALANCE_ELEVATOR_MM_PORT = 0;
    public static final int BALANCE_ELEVATOR_PID_PORT = 1;
    public static final int CLIMB_ELEVATOR_MM_PORT = 0;
    public static final int HOOD_MM_PORT = 0;

    public static final double MIN_CLIMB_ELEVATOR_PERCENT_BUS = 0.1;
    public static final double MIN_BALANCE_ELEVATOR_PERCENT_BUS = 0.06;

    public static final double ELEVATOR_AUTO_ZERO_SPEED = -0.3;

    public static final int INDEXER_DIO_PORT = 0;

}
