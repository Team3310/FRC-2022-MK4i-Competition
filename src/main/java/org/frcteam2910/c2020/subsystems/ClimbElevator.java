package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import org.frcteam2910.c2020.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbElevator extends SubsystemBase {

    public enum ClimbControlMode{
        MANUAL, MOTION_MAGIC
    }

    // Motor Controllers
    private TalonFX elevatorMotor;

    // Misc
    private ClimbControlMode controlMode = ClimbControlMode.MANUAL;
    private double targetPositionTicks = 0;
    private double manualElevatorSpeed = 0;

    //Conversions
    private static final double PULLEY_DIAMETER_INCHES = 1.923;
    private static final double ELEVATOR_OUTPUT_TO_ENCODER_RATIO = (54 / 18) * (54 / 12);
    private static final double ELEVATOR_ROTATIONS_TO_INCHES = Math.PI * PULLEY_DIAMETER_INCHES;
    private static final double ELEVATOR_INCHES_TO_ENCODER_TICKS = ELEVATOR_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / ELEVATOR_ROTATIONS_TO_INCHES;

    private final static ClimbElevator INSTANCE = new ClimbElevator();

    private ClimbElevator() {
        elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR_ID, "Drivetrain");

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        elevatorMotor.configAllSettings(configs);

        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.configMotionCruiseVelocity(6000);
        elevatorMotor.configMotionAcceleration(14000);
        elevatorMotor.configMotionSCurveStrength(4);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;
        statorCurrentConfigs.enable = false;
        elevatorMotor.configStatorCurrentLimit(statorCurrentConfigs);

        elevatorMotor.config_kF(Constants.CLIMB_ELEVATOR_MM_PORT, 0.055);
        elevatorMotor.config_kP(Constants.CLIMB_ELEVATOR_MM_PORT, 0.20); //0.1
        elevatorMotor.config_kI(Constants.CLIMB_ELEVATOR_MM_PORT, 0.0001);
        elevatorMotor.config_kD(Constants.CLIMB_ELEVATOR_MM_PORT, 0.0);
    }

    public static ClimbElevator getInstance() {
        return INSTANCE;
    }


    private double limitElevatorInches(double targetInches) {
        if (targetInches < Constants.ELEVATOR_MIN_INCHES) {
            return Constants.ELEVATOR_MIN_INCHES;
        } else if (targetInches > Constants.ELEVATOR_MAX_INCHES) {
            return Constants.ELEVATOR_MAX_INCHES;
        }

        return targetInches;
    }

    public double getElevatorRotations(){
        return elevatorMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / ELEVATOR_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getElevatorInches(){
        return getElevatorRotations() * ELEVATOR_ROTATIONS_TO_INCHES;
    }

    public double getElevatorEncoderTicksAbsolute(double inches){
        return (int) (inches * ELEVATOR_INCHES_TO_ENCODER_TICKS);
    }

    public void setElevatorZero(){
        elevatorMotor.setSelectedSensorPosition(0);
    }

    public synchronized void setElevatorMotionMagicPositionAbsolute(double inches) {
        controlMode = ClimbControlMode.MOTION_MAGIC;
        elevatorMotor.selectProfileSlot(1, 0);
        targetPositionTicks = getElevatorEncoderTicksAbsolute(limitElevatorInches(inches));
        elevatorMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setElevatorSpeed(double speed) {
        controlMode = ClimbControlMode.MANUAL;
        manualElevatorSpeed = speed;
        double curSpeed = speed;
        if (getElevatorInches() < Constants.ELEVATOR_MIN_INCHES && speed < 0.0) {
            curSpeed = 0;
        } else if (getElevatorInches() > Constants.ELEVATOR_MAX_INCHES && speed > 0.0) {
            curSpeed = 0;
        }
        
        elevatorMotor.set(ControlMode.PercentOutput, curSpeed);

    }

    public synchronized void setHoldElevator(){
        setElevatorMotionMagicPositionAbsolute(getElevatorInches());
    }

    @Override
    public void periodic() {
        if (controlMode == ClimbControlMode.MANUAL) {

            if (getElevatorInches() < Constants.ELEVATOR_MIN_INCHES && manualElevatorSpeed < 0.0) {
                setHoldElevator();
            } else if (getElevatorInches() > Constants.ELEVATOR_MAX_INCHES && manualElevatorSpeed > 0.0) {
                setHoldElevator();
            }
        }

        SmartDashboard.putNumber("Elevator Climb position", getElevatorInches());
    }
}

