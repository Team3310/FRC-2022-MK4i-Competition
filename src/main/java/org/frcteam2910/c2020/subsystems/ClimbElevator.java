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
import org.frcteam2910.c2020.util.Util;


public class ClimbElevator extends SubsystemBase {

    public static final double AUTO_ZERO_MOTOR_CURRENT = 1.0;


    public enum ClimbControlMode{
        MANUAL, MOTION_MAGIC
    }

    private boolean isZeroing = false;

    // Motor Controllers
    private TalonFX elevatorMotorMaster;
    private TalonFX elevatorMotorSlave;

    // Misc
    private ClimbControlMode controlMode = ClimbControlMode.MANUAL;
    private double targetPositionTicks = 0;
    private double manualElevatorSpeed = 0;
    private double positionOffset = 0;

    //Conversions
    private static final double PULLEY_DIAMETER_INCHES = 1.163;
    private static final double ELEVATOR_OUTPUT_TO_ENCODER_RATIO = (50 / 12) * (50 / 22); //Practice bot Settings
    //private static final double ELEVATOR_OUTPUT_TO_ENCODER_RATIO = (58 / 14) * (54 / 12); //Comp bot settings
    private static final double ELEVATOR_ROTATIONS_TO_INCHES = Math.PI * PULLEY_DIAMETER_INCHES;
    private static final double ELEVATOR_INCHES_TO_ENCODER_TICKS = ELEVATOR_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / ELEVATOR_ROTATIONS_TO_INCHES;

    private final static ClimbElevator INSTANCE = new ClimbElevator();

    private ClimbElevator() {
        elevatorMotorMaster = new TalonFX(Constants.ELEVATOR_MOTOR_MASTER_ID, "Drivetrain");
        elevatorMotorSlave = new TalonFX(Constants.ELEVATOR_MOTOR_SLAVE_ID, "Drivetrain");

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        elevatorMotorMaster.configAllSettings(configs);
        elevatorMotorSlave.configAllSettings(configs);

        elevatorMotorSlave.follow(elevatorMotorMaster);

        elevatorMotorMaster.setNeutralMode(NeutralMode.Brake);
        elevatorMotorMaster.configMotionCruiseVelocity(6000);
        elevatorMotorMaster.configMotionAcceleration(14000);
        elevatorMotorMaster.configMotionSCurveStrength(4);

        elevatorMotorSlave.setNeutralMode(NeutralMode.Brake);
        elevatorMotorSlave.configMotionCruiseVelocity(6000);
        elevatorMotorSlave.configMotionAcceleration(14000);
        elevatorMotorSlave.configMotionSCurveStrength(4);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 60;
        statorCurrentConfigs.enable = true;
        elevatorMotorMaster.configStatorCurrentLimit(statorCurrentConfigs);
        elevatorMotorSlave.configStatorCurrentLimit(statorCurrentConfigs);

        elevatorMotorMaster.config_kF(Constants.CLIMB_ELEVATOR_MM_PORT, 0.055);
        elevatorMotorMaster.config_kP(Constants.CLIMB_ELEVATOR_MM_PORT, 0.20); //0.1
        elevatorMotorMaster.config_kI(Constants.CLIMB_ELEVATOR_MM_PORT, 0.0001);
        elevatorMotorMaster.config_kD(Constants.CLIMB_ELEVATOR_MM_PORT, 0.0);
    }

    public static ClimbElevator getInstance() {
        return INSTANCE;
    }

    public ClimbControlMode getControlMode(){
        return controlMode;
    }

    private double limitElevatorInches(double targetInches) {
        if (targetInches < Constants.ELEVATOR_MIN_INCHES) {
            return Constants.ELEVATOR_MIN_INCHES;
        } else if (targetInches > Constants.ELEVATOR_MAX_INCHES) {
            return Constants.ELEVATOR_MAX_INCHES;
        }

        return targetInches;
    }

    public void setZeroing(boolean zeroing) {
        isZeroing = zeroing;
    }

    public void setControlMode(ClimbControlMode controlMode){
        this.controlMode = controlMode;
    }

    public double getElevatorRotations(){
        return elevatorMotorMaster.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / ELEVATOR_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getElevatorInches(){
        return (getElevatorRotations() * ELEVATOR_ROTATIONS_TO_INCHES) + positionOffset;
    }

    public double getElevatorEncoderTicksAbsolute(double inches){
        return (int) (inches * ELEVATOR_INCHES_TO_ENCODER_TICKS);
    }

    public void setElevatorZero(double offset){
        positionOffset = offset;
        elevatorMotorMaster.setSelectedSensorPosition(0);
    }

    public synchronized void setElevatorMotionMagicPositionAbsolute(double inches) {
        controlMode = ClimbControlMode.MOTION_MAGIC;
        elevatorMotorMaster.selectProfileSlot(1, 0);
        targetPositionTicks = getElevatorEncoderTicksAbsolute(limitElevatorInches(inches - positionOffset));
        elevatorMotorMaster.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setElevatorSpeed(double speed) {
        if(!isZeroing) {
            manualElevatorSpeed = speed;
            double curSpeed = speed;

            controlMode = ClimbControlMode.MANUAL;
            if (getElevatorInches() < Constants.ELEVATOR_MIN_INCHES && speed < 0.0) {
                curSpeed = 0;
            } else if (getElevatorInches() > Constants.ELEVATOR_MAX_INCHES && speed > 0.0) {
                curSpeed = 0;
            }

            elevatorMotorMaster.set(ControlMode.PercentOutput, curSpeed);
        }
    }

    public synchronized void setElevatorSpeedZeroing(double speed) {
        elevatorMotorMaster.set(ControlMode.PercentOutput, speed);
    }

    public double getElevatorMotorCurrent(){
        return elevatorMotorMaster.getStatorCurrent();
    }

    public synchronized boolean hasFinishedHoodTrajectory() {
        return controlMode == ClimbControlMode.MOTION_MAGIC
                && Util.epsilonEquals(elevatorMotorMaster.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    public synchronized void setHoldElevator(){
        if(!isZeroing) {
            setElevatorMotionMagicPositionAbsolute(getElevatorInches());
        }
    }

    @Override
    public void periodic() {
        if(!isZeroing) {
            if (controlMode == ClimbControlMode.MANUAL) {

                if (getElevatorInches() < Constants.ELEVATOR_MIN_INCHES && manualElevatorSpeed < 0.0) {
                    setHoldElevator();
                } else if (getElevatorInches() > Constants.ELEVATOR_MAX_INCHES && manualElevatorSpeed > 0.0) {
                    setHoldElevator();
                }
            }

            //SmartDashboard.putNumber("Climb Height", getElevatorInches());
        }
    }
}

