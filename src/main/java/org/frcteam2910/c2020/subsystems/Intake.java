package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.util.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.PlaystationController;
import org.frcteam2910.common.robot.input.XboxController;


public class Intake extends SubsystemBase {

    public enum LiftControlMode{
        PID, MOTION_MAGIC, CURRENT
    }

    // Conversions
    private static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 40.0 / 10.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    private final double LIFT_OUTPUT_TO_ENCODER_RATIO = 48.0 / 10.0 * 54.0 / 14.0 * 7.0;
    private final double LIFT_REVOLUTIONS_TO_ENCODER_TICKS = LIFT_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private final double LIFT_DEGREES_TO_ENCODER_TICKS = LIFT_REVOLUTIONS_TO_ENCODER_TICKS / 360.0;
    private double homePositionAngleDegrees = Constants.LIFT_COMPETITION_HOME_POSITION_DEGREES;

    // Motor Controllers
    private TalonFX intakeMotor;
    private TalonFX liftMotor;


    // Misc
    private LiftControlMode liftControlMode = LiftControlMode.MOTION_MAGIC;
    private static final int kIntakeVelocitySlot = 0;
    private Controller secondaryController;
    private boolean hasSetIntakeZero = true;
    private double targetPositionTicks = 0;
    public static final double AUTO_ZERO_MOTOR_CURRENT = 1.0;
    private boolean isLifting = false;

    private final static Intake INSTANCE = new Intake();

    private Intake() {
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, "Drivetrain");
        liftMotor = new TalonFX(Constants.INTAKE_LIFT_MOTOR_CAN_ID);

        intakeMotor.configFactoryDefault();

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        intakeMotor.configAllSettings(configs);
        liftMotor.configAllSettings(configs);

        intakeMotor.setInverted(TalonFXInvertType.CounterClockwise);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        liftMotor.setInverted(TalonFXInvertType.Clockwise);
        liftMotor.setNeutralMode(NeutralMode.Brake);
        liftMotor.configMotionCruiseVelocity(6000);
        liftMotor.configMotionAcceleration(12000);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;
        statorCurrentConfigs.enable = false;
        intakeMotor.configStatorCurrentLimit(statorCurrentConfigs);

        final StatorCurrentLimitConfiguration liftStatorCurrentConfigs = new StatorCurrentLimitConfiguration();
        liftStatorCurrentConfigs.currentLimit = 20;
        liftStatorCurrentConfigs.enable = true;
        liftMotor.configStatorCurrentLimit(liftStatorCurrentConfigs);

        liftMotor.setNeutralMode(NeutralMode.Coast);

        intakeMotor.config_kF(kIntakeVelocitySlot, 0.055);
        intakeMotor.config_kP(kIntakeVelocitySlot, 0.10);
        intakeMotor.config_kI(kIntakeVelocitySlot, 0.0001);
        intakeMotor.config_kD(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.RollerRPMToNativeUnits(200));

        liftMotor.config_kF(Constants.LIFT_MM_PORT, 0.0);
        liftMotor.config_kP(Constants.LIFT_MM_PORT, 0.0);//.9
        liftMotor.config_kI(Constants.LIFT_MM_PORT, 0.0);//.008
        liftMotor.config_kD(Constants.LIFT_MM_PORT, 0.0);

        liftMotor.config_kF(Constants.LIFT_PID_PORT, 0.0);
        liftMotor.config_kP(Constants.LIFT_PID_PORT, 0.03);//.9
        liftMotor.config_kI(Constants.LIFT_PID_PORT, 0.0);//.008
        liftMotor.config_kD(Constants.LIFT_PID_PORT, 0.0);
    }

    public static Intake getInstance() {
        return INSTANCE;
    }

    private Axis getRightTriggerAxis(){return secondaryController.getRightTriggerAxis();}

    private Axis getLeftTriggerAxis(){return secondaryController.getLeftTriggerAxis();}

    public void setController(XboxController xboxController){
        secondaryController = xboxController;
    }

    public void setController(PlaystationController playstationController){
        secondaryController = playstationController;
    }

    public void setRollerSpeed(double speed) {
        this.intakeMotor.set(ControlMode.PercentOutput, speed);
        //System.out.println("Set Intake Speed = " + speed);
    }

    public void resetIntakeEncoder() {
        this.intakeMotor.setSelectedSensorPosition(0);
    }

    public double getRollerRotations() {
        return intakeMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getRollerRPM() {
        return intakeMotor.getSelectedSensorVelocity() / INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }

    public void setRollerRPM(double rpm) {
        this.intakeMotor.set(ControlMode.Velocity, this.RollerRPMToNativeUnits(rpm));
        //System.out.println("Set Intake RPM = " + rpm);

    }

    public void variableIntakeRPM(){


        if(getRightTriggerAxis().getButton(0.1).getAsBoolean()){
            setRollerRPM(getRightTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
            hasSetIntakeZero = false;
        }
        else if(getLeftTriggerAxis().getButton(0.1).getAsBoolean()){
            setRollerRPM( -getLeftTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
            hasSetIntakeZero = false;
        }
        else{
            if(!hasSetIntakeZero){
                setRollerSpeed(0);
                hasSetIntakeZero = true;
            }
        }
    }

    public double RollerRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public void setLiftSpeed(double speed){
        liftMotor.set(ControlMode.PercentOutput, speed);
    }

    private double getLiftEncoderTicksAbsolute(double angle) {
        double positionDegrees = angle - homePositionAngleDegrees;
        return (int) (positionDegrees * LIFT_DEGREES_TO_ENCODER_TICKS);
    }


    private synchronized void setLiftControlMode(LiftControlMode controlMode) {
        this.liftControlMode = controlMode;
    }

    private synchronized LiftControlMode getLiftControlMode() {
        return this.liftControlMode;
    }

    public void resetLiftHomePosition() {
        liftMotor.setSelectedSensorPosition(0);
    }

    public double getLiftAngleAbsoluteDegrees() {
        return (double) liftMotor.getSelectedSensorPosition() / LIFT_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees;
    }

    public double getLiftCurrent(){
        return liftMotor.getStatorCurrent();
    }

    private double limitLiftAngle(double targetAngle) {
        if (targetAngle < Constants.LIFT_MIN_ANGLE_DEGREES) {
            return Constants.LIFT_MIN_ANGLE_DEGREES;
        } else if (targetAngle > Constants.LIFT_MAX_ANGLE_DEGREES) {
            return Constants.LIFT_MAX_ANGLE_DEGREES;
        }

        return targetAngle;
    }

    // Motion Magic
    public synchronized void setLiftMotionMagicPositionAbsolute(double angle) {
        if (getLiftControlMode() != LiftControlMode.MOTION_MAGIC) {
            setLiftControlMode(LiftControlMode.MOTION_MAGIC);
        }
        setLiftMotionMagicPositionAbsoluteInternal(angle);
    }

    public synchronized void setLiftMotionMagicPositionAbsoluteInternal(double angle) {
        liftMotor.selectProfileSlot(Constants.LIFT_MM_PORT, 0);
        double limitedAngle = limitLiftAngle(angle);
        targetPositionTicks = getLiftEncoderTicksAbsolute(limitedAngle);
        liftMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setLiftPIDPositionAbsoluteInternal(double angle) {
        setLiftControlMode(LiftControlMode.PID);
        if(angle > 10){
            isLifting = true;
        }
        else{
            isLifting = false;
        }
        liftMotor.selectProfileSlot(Constants.LIFT_PID_PORT, 0);
        double limitedAngle = limitLiftAngle(angle);
        targetPositionTicks = getLiftEncoderTicksAbsolute(limitedAngle);
        liftMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized boolean hasFinishedLiftTrajectory() {
        return liftControlMode == LiftControlMode.MOTION_MAGIC
                && Util.epsilonEquals(liftMotor.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    public void setLiftCoast() {
        liftMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setLiftBrake() {
        liftMotor.setNeutralMode(NeutralMode.Brake);
    }


    @Override
    public void periodic(){
        variableIntakeRPM();
        if(getLiftControlMode() != LiftControlMode.CURRENT && !isLifting && getLiftAngleAbsoluteDegrees() < 10.0){
            setLiftControlMode(LiftControlMode.CURRENT);
            liftMotor.set(TalonFXControlMode.PercentOutput, -0.05);
        }
//        SmartDashboard.putNumber("Intake Amperage", intakeMotor.getStatorCurrent());
//        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());
//        SmartDashboard.putNumber("Intake lift Angle", getLiftAngleAbsoluteDegrees());
//        SmartDashboard.putNumber("Intake lift Current", getLiftCurrent());
    }

//        SmartDashboard.putNumber("Intake Roller Rotations", this.getRollerRotations());
//        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());

}

