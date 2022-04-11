package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import org.frcteam2910.c2020.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BalanceElevator extends SubsystemBase {


    public enum BalanceControlMode{
        MANUAL, MOTION_MAGIC, PID, ZERO
    }

    // Motor Controllers
    private TalonFX balanceElevatorMotor;

    // Misc
    private BalanceControlMode controlMode = BalanceControlMode.MANUAL;
    private double targetPositionTicks = 0;
    private double manualBalanceElevatorSpeed = 0;
    private boolean sysStatus = false;

    //Conversions
    private static final double PULLEY_DIAMETER_INCHES = 2;
    private static final double BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO = 50 / 11;
    private static final double BALANCE_ELEVATOR_ROTATIONS_TO_INCHES = Math.PI * PULLEY_DIAMETER_INCHES;
    private static final double BALANCE_ELEVATORS_INCHES_TO_ENCODER_TICKS = BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / BALANCE_ELEVATOR_ROTATIONS_TO_INCHES;
 

    private final static BalanceElevator INSTANCE = new BalanceElevator();

    private BalanceElevator() {

        balanceElevatorMotor = new TalonFX(Constants.BALANCE_ELEVATOR_ID, "Drivetrain");


        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        balanceElevatorMotor.configAllSettings(configs);

        balanceElevatorMotor.setNeutralMode(NeutralMode.Brake);
        balanceElevatorMotor.configMotionCruiseVelocity(10000);
        balanceElevatorMotor.configMotionAcceleration(28000);
        balanceElevatorMotor.configMotionSCurveStrength(4);

        balanceElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;
        statorCurrentConfigs.enable = false;
        balanceElevatorMotor.configStatorCurrentLimit(statorCurrentConfigs);

        balanceElevatorMotor.config_kF(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0); //.055
        balanceElevatorMotor.config_kP(Constants.BALANCE_ELEVATOR_MM_PORT, 0.40);
        balanceElevatorMotor.config_kI(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0001);
        balanceElevatorMotor.config_kD(Constants.BALANCE_ELEVATOR_MM_PORT, 0.0);

        balanceElevatorMotor.config_kF(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0); //.055
        balanceElevatorMotor.config_kP(Constants.BALANCE_ELEVATOR_PID_PORT, 0.40);
        balanceElevatorMotor.config_kI(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0001);
        balanceElevatorMotor.config_kD(Constants.BALANCE_ELEVATOR_PID_PORT, 0.0);
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

    public double getBalanceElevatorRotations(){
        return balanceElevatorMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / BALANCE_ELEVATOR_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getBalanceElevatorInches(){
        return getBalanceElevatorRotations() * BALANCE_ELEVATOR_ROTATIONS_TO_INCHES;
    }

    public double getBalanceElevatorEncoderTicksAbsolute(double inches){
        return (int) (inches * BALANCE_ELEVATORS_INCHES_TO_ENCODER_TICKS);
    }

    public void setElevatorZero(){
        balanceElevatorMotor.setSelectedSensorPosition(0);
    }

    public synchronized void setBalanceElevatorMotionMagicPositionAbsolute(double inches) {
        controlMode = BalanceControlMode.MOTION_MAGIC;
        balanceElevatorMotor.selectProfileSlot(Constants.BALANCE_ELEVATOR_MM_PORT, 0);
        targetPositionTicks = getBalanceElevatorEncoderTicksAbsolute(limitBalanceElevator(inches));
        balanceElevatorMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setBalanceElevatorHoldPID(double inches){
        controlMode = BalanceControlMode.PID;
        balanceElevatorMotor.selectProfileSlot(Constants.BALANCE_ELEVATOR_PID_PORT, 0);
        targetPositionTicks = getBalanceElevatorEncoderTicksAbsolute(limitBalanceElevator(inches));
        balanceElevatorMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setBalanceElevatorSpeed(double speed) {
        if(controlMode != BalanceControlMode.ZERO){
            controlMode = BalanceControlMode.MANUAL;
        }

        manualBalanceElevatorSpeed = speed;
    }

    public synchronized void setHoldBalanceElevator(){
        setBalanceElevatorHoldPID(getBalanceElevatorInches());
    }

    @Override
    public void periodic() {
        if (controlMode == BalanceControlMode.MANUAL) {
            if (getBalanceElevatorInches() < Constants.BALANCE_ELEVATOR_MIN_INCHES && manualBalanceElevatorSpeed < 0.0) {
            } else if (getBalanceElevatorInches() > Constants.BALANCE_ELEVATOR_MAX_INCHES && manualBalanceElevatorSpeed > 0.0) {
            }
            else{
                balanceElevatorMotor.set(ControlMode.PercentOutput, manualBalanceElevatorSpeed);
            }
        }

        //SmartDashboard.putNumber("Balance Elevator Height", getBalanceElevatorInches());

    }
}

