package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BalanceElevator extends SubsystemBase {


    public enum BalanceControlMode{
        MANUAL, MOTION_MAGIC, PID, ZERO
    }

    // Motor Controllers
    private TalonFX balanceElevatorRightMotor;
    private TalonFX balanceElevatorLeftMotor;

    // Misc
    private BalanceControlMode controlMode = BalanceControlMode.MANUAL;
    private double manualBalanceElevatorSpeed = 0;
    private boolean sysStatus = false;

    //Conversions
    private static final double PULLEY_DIAMETER_INCHES = 2;
    private static final double BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO = 50 / 11;
    private static final double BALANCE_ELEVATOR_ROTATIONS_TO_INCHES = Math.PI * PULLEY_DIAMETER_INCHES;
    private static final double BALANCE_ELEVATORS_INCHES_TO_ENCODER_TICKS = BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / BALANCE_ELEVATOR_ROTATIONS_TO_INCHES;
 

    private final static BalanceElevator INSTANCE = new BalanceElevator();

    private BalanceElevator() {

        balanceElevatorRightMotor = new TalonFX(Constants.BALANCE_ELEVATOR_MASTER_ID, "Drivetrain");
        balanceElevatorLeftMotor = new TalonFX(Constants.BALANCE_ELEVATOR_SLAVE_ID, "Drivetrain");


        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        balanceElevatorRightMotor.configAllSettings(configs);
        balanceElevatorLeftMotor.configAllSettings(configs);

        balanceElevatorRightMotor.setNeutralMode(NeutralMode.Brake);
        balanceElevatorRightMotor.configMotionCruiseVelocity(10000);
        balanceElevatorRightMotor.configMotionAcceleration(28000);
        balanceElevatorRightMotor.configMotionSCurveStrength(4);

        balanceElevatorLeftMotor.setNeutralMode(NeutralMode.Brake);
        balanceElevatorLeftMotor.configMotionCruiseVelocity(10000);
        balanceElevatorLeftMotor.configMotionAcceleration(28000);
        balanceElevatorLeftMotor.configMotionSCurveStrength(4);

        balanceElevatorRightMotor.setInverted(TalonFXInvertType.Clockwise);
        balanceElevatorLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;

        statorCurrentConfigs.enable = false;
        balanceElevatorRightMotor.configStatorCurrentLimit(statorCurrentConfigs);
        balanceElevatorLeftMotor.configStatorCurrentLimit(statorCurrentConfigs);

        balanceElevatorRightMotor.config_kF(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0); //.055
        balanceElevatorRightMotor.config_kP(Constants.BALANCE_ELEVATOR_MM_PORT, 0.40);
        balanceElevatorRightMotor.config_kI(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0001);
        balanceElevatorRightMotor.config_kD(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0);

        balanceElevatorRightMotor.config_kF(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0); //.055
        balanceElevatorRightMotor.config_kP(Constants.BALANCE_ELEVATOR_PID_PORT, 0.40);
        balanceElevatorRightMotor.config_kI(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0001);
        balanceElevatorRightMotor.config_kD(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0);

        balanceElevatorLeftMotor.config_kF(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0); //.055
        balanceElevatorLeftMotor.config_kP(Constants.BALANCE_ELEVATOR_MM_PORT, 0.40);
        balanceElevatorLeftMotor.config_kI(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0001);
        balanceElevatorLeftMotor.config_kD(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0);

        balanceElevatorLeftMotor.config_kF(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0); //.055
        balanceElevatorLeftMotor.config_kP(Constants.BALANCE_ELEVATOR_PID_PORT, 0.40);
        balanceElevatorLeftMotor.config_kI(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0001);
        balanceElevatorLeftMotor.config_kD(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0);
    }

    public static BalanceElevator getInstance() {
        return INSTANCE;
    }

    public BalanceControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(BalanceControlMode controlMode){
        this.controlMode = controlMode;
    }

    private double limitBalanceElevator(double targetInches) {
        if (targetInches < Constants.BALANCE_ELEVATOR_MIN_INCHES) {
            return Constants.BALANCE_ELEVATOR_MIN_INCHES;
        } else if (targetInches > Constants.BALANCE_ELEVATOR_MAX_INCHES) {
            return Constants.BALANCE_ELEVATOR_MAX_INCHES;
        }

        return targetInches;
    }

    public void setBalanceElevatorStatus(boolean status) {
        sysStatus = status;
    }
    public boolean getBalanceElevatorStatus() {
        return sysStatus;
    }

    public double getBalanceElevatorRotationsLeft(){
        return balanceElevatorLeftMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getBalanceElevatorInchesLeft(){
        return getBalanceElevatorRotationsLeft() * BALANCE_ELEVATOR_ROTATIONS_TO_INCHES;
    }

    public double getBalanceElevatorRotationsRight(){
        return balanceElevatorRightMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getBalanceElevatorInchesRight(){
        return getBalanceElevatorRotationsRight() * BALANCE_ELEVATOR_ROTATIONS_TO_INCHES;
    }

    public double getBalanceElevatorEncoderTicksAbsolute(double inches){
        return (int) (inches * BALANCE_ELEVATORS_INCHES_TO_ENCODER_TICKS);
    }

    public void setElevatorZero(){
        balanceElevatorRightMotor.setSelectedSensorPosition(0);
        balanceElevatorLeftMotor.setSelectedSensorPosition(0);
    }

    public synchronized void setBalanceElevatorMotionMagicPositionAbsoluteRight(double inches) {
        controlMode = BalanceControlMode.MOTION_MAGIC;
        balanceElevatorRightMotor.selectProfileSlot(Constants.BALANCE_ELEVATOR_MM_PORT, 0);
        double targetPositionTicks = getBalanceElevatorEncoderTicksAbsolute(limitBalanceElevator(inches));
        balanceElevatorRightMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setBalanceElevatorMotionMagicPositionAbsoluteLeft(double inches) {
        controlMode = BalanceControlMode.MOTION_MAGIC;
        balanceElevatorLeftMotor.selectProfileSlot(Constants.BALANCE_ELEVATOR_MM_PORT, 0);
        double targetPositionTicks = getBalanceElevatorEncoderTicksAbsolute(limitBalanceElevator(inches));
        balanceElevatorLeftMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setBalanceElevatorHoldPIDRight(double inches){
        balanceElevatorRightMotor.selectProfileSlot(Constants.BALANCE_ELEVATOR_PID_PORT, 0);
        double targetPositionTicks = getBalanceElevatorEncoderTicksAbsolute(limitBalanceElevator(inches));
        balanceElevatorRightMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setBalanceElevatorHoldPIDLeft(double inches){
        balanceElevatorLeftMotor.selectProfileSlot(Constants.BALANCE_ELEVATOR_PID_PORT, 0);
        double targetPositionTicks = getBalanceElevatorEncoderTicksAbsolute(limitBalanceElevator(inches));
        balanceElevatorLeftMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setBalanceElevatorSpeed(double speed) {
        if(controlMode != BalanceControlMode.ZERO){
            controlMode = BalanceControlMode.MANUAL;
        }

        manualBalanceElevatorSpeed = speed;
    }

    public synchronized void setHoldBalanceElevator(){
        controlMode = BalanceControlMode.PID;
        setBalanceElevatorHoldPIDRight(getBalanceElevatorInchesRight());
        setBalanceElevatorHoldPIDLeft(getBalanceElevatorInchesLeft());
    }

    @Override
    public void periodic() {
        if (controlMode == BalanceControlMode.MANUAL) {
            if (getBalanceElevatorInchesRight() < Constants.BALANCE_ELEVATOR_MIN_INCHES && manualBalanceElevatorSpeed < 0.0) {
                setBalanceElevatorHoldPIDRight(getBalanceElevatorInchesRight());
            } else if (getBalanceElevatorInchesRight() > Constants.BALANCE_ELEVATOR_MAX_INCHES && manualBalanceElevatorSpeed > 0.0) {
                setBalanceElevatorHoldPIDRight(getBalanceElevatorInchesRight());
            }
            else{
                balanceElevatorRightMotor.set(ControlMode.PercentOutput, manualBalanceElevatorSpeed);
            }

            if (getBalanceElevatorInchesLeft() < Constants.BALANCE_ELEVATOR_MIN_INCHES && manualBalanceElevatorSpeed < 0.0) {
                setBalanceElevatorHoldPIDLeft(getBalanceElevatorInchesLeft());
            } else if (getBalanceElevatorInchesLeft() > Constants.BALANCE_ELEVATOR_MAX_INCHES && manualBalanceElevatorSpeed > 0.0) {
                setBalanceElevatorHoldPIDLeft(getBalanceElevatorInchesLeft());
            }
            else{
                balanceElevatorLeftMotor.set(ControlMode.PercentOutput, manualBalanceElevatorSpeed);
            }
        }

        SmartDashboard.putNumber("Left Balance Elevator Height", getBalanceElevatorInchesLeft());
        SmartDashboard.putNumber("Right Balance Elevator Height", getBalanceElevatorInchesRight());

    }
}

