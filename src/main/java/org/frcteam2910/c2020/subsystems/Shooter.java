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
import org.frcteam2910.c2020.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    public enum HoodControlMode {
        MANUAL, MOTION_MAGIC
    };

    // Motor Controllers
    private TalonFX shooterMotorMaster;
    private TalonFX shooterMotorSlave;
    private TalonFX hoodMotor;

    //Conversions
    private static final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 1.0;
    public static final double SHOOTER_REVOLUTIONS_TO_ENCODER_TICKS = SHOOTER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    private final double HOOD_OUTPUT_TO_ENCODER_RATIO = 4.0 * 392.0 / 18.0;
    private final double HOOD_REVOLUTIONS_TO_ENCODER_TICKS = HOOD_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private final double HOOD_DEGREES_TO_ENCODER_TICKS = HOOD_REVOLUTIONS_TO_ENCODER_TICKS / 360.0;
    private double homePositionAngleDegrees = Constants.HOOD_COMPETITION_HOME_POSITION_DEGREES;

    // Misc
    private double targetPositionTicks = 0.0;
    private HoodControlMode hoodControlMode = HoodControlMode.MANUAL;
    private boolean isReady;
    private boolean hoodReset = false;

    private final static Shooter INSTANCE = new Shooter();

    private Shooter() {
        shooterMotorMaster = new TalonFX(Constants.SHOOTER_MASTER_CAN_ID);
        shooterMotorSlave = new TalonFX(Constants.SHOOTER_SLAVE_CAN_ID);
        hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        shooterMotorMaster.configAllSettings(configs);
        shooterMotorSlave.configAllSettings(configs);
        hoodMotor.configAllSettings(configs);

        shooterMotorMaster.setInverted(TalonFXInvertType.CounterClockwise);
        shooterMotorMaster.setNeutralMode(NeutralMode.Coast);

        shooterMotorSlave.setInverted(TalonFXInvertType.Clockwise);
        shooterMotorSlave.setNeutralMode(NeutralMode.Coast);
        shooterMotorSlave.follow(shooterMotorMaster);

        hoodMotor.setInverted(TalonFXInvertType.CounterClockwise);
        hoodMotor.setNeutralMode(NeutralMode.Brake);
        hoodMotor.configMotionCruiseVelocity(6000);
        hoodMotor.configMotionAcceleration(12000);

    
        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 60;
        statorCurrentConfigs.enable = true;
        statorCurrentConfigs.triggerThresholdCurrent = 80;
        statorCurrentConfigs.triggerThresholdTime = 0.5;
        shooterMotorMaster.configStatorCurrentLimit(statorCurrentConfigs);
        shooterMotorSlave.configStatorCurrentLimit(statorCurrentConfigs);

        StatorCurrentLimitConfiguration statorCurrentHoodConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentHoodConfigs.currentLimit = 30;
        statorCurrentHoodConfigs.enable = true;
        statorCurrentHoodConfigs.triggerThresholdCurrent = 60;
        statorCurrentHoodConfigs.triggerThresholdTime = 0.5;
        hoodMotor.configStatorCurrentLimit(statorCurrentHoodConfigs);

        shooterMotorMaster.config_kF(0, 0.05);
        shooterMotorMaster.config_kP(0, 0.5);
        shooterMotorMaster.config_kI(0, 0);
        shooterMotorMaster.config_kD(0, 0); 

        hoodMotor.config_kF(Constants.HOOD_MM_PORT, 0.045);
        hoodMotor.config_kP(Constants.HOOD_MM_PORT, 0.5);//.9
        hoodMotor.config_kI(Constants.HOOD_MM_PORT, 0.008);//.008
        hoodMotor.config_kD(Constants.HOOD_MM_PORT, 0.0);
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    public void setShooterSpeed(double speed) {
        this.shooterMotorMaster.set(ControlMode.PercentOutput, speed);
    }

    public double getShooterRPM() {
        return shooterMotorMaster.getSelectedSensorVelocity() / SHOOTER_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }

    public void setShooterRPM(double rpm) {
        this.shooterMotorMaster.set(ControlMode.Velocity, this.shooterRPMToNativeUnits(rpm));
    }

    public double shooterRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public double kickerRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    private double limitHoodAngle(double targetAngle) {
        if (targetAngle < Constants.HOOD_MIN_ANGLE_DEGREES) {
            return Constants.HOOD_MIN_ANGLE_DEGREES;
        } else if (targetAngle > Constants.HOOD_MAX_ANGLE_DEGREES) {
            return Constants.HOOD_MAX_ANGLE_DEGREES;
        }

        return targetAngle;
    }

    private double getHoodEncoderTicksAbsolute(double angle) {
        double positionDegrees = angle - homePositionAngleDegrees;
        return (int) (positionDegrees * HOOD_DEGREES_TO_ENCODER_TICKS);
    }


    private synchronized void setHoodControlMode(HoodControlMode controlMode) {
        this.hoodControlMode = controlMode;
    }

    private synchronized Shooter.HoodControlMode getHoodControlMode() {
        return this.hoodControlMode;
    }

    public void resetHoodHomePosition() {
        System.out.println("Zero Hood");
        hoodReset = true;
        hoodMotor.setSelectedSensorPosition(0);
    }


    public double getHoodAngleAbsoluteDegrees() {
        return (double)hoodMotor.getSelectedSensorPosition() / HOOD_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees;
    }


    // Motion Magic
    public synchronized void setHoodMotionMagicPositionAbsolute(double angle) {
        if (getHoodControlMode() != HoodControlMode.MOTION_MAGIC) {
            setHoodControlMode(HoodControlMode.MOTION_MAGIC);
        }
        setHoodMotionMagicPositionAbsoluteInternal(angle);
    }

    public synchronized void setHoodMotionMagicPositionAbsoluteInternal(double angle) {
        hoodMotor.selectProfileSlot(Constants.HOOD_MM_PORT, 0);
        double limitedAngle = limitHoodAngle(angle);
        targetPositionTicks = getHoodEncoderTicksAbsolute(limitedAngle);
        hoodMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.07);
    }

    public synchronized void setHoodPIDPositionAbsoluteInternal(double angle) {
        hoodMotor.selectProfileSlot(Constants.HOOD_MM_PORT, 0);
        double limitedAngle = limitHoodAngle(angle);
        targetPositionTicks = getHoodEncoderTicksAbsolute(limitedAngle);
        hoodMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.07);
    }

    public synchronized boolean hasFinishedHoodTrajectory() {
        return hoodControlMode == HoodControlMode.MOTION_MAGIC
                && Util.epsilonEquals(hoodMotor.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    public void setReady(boolean isReady) {
        this.isReady = isReady;
    }

    public boolean isReady() {
        return isReady;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Angle", getHoodAngleAbsoluteDegrees());
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putBoolean("Hood Reset", hoodReset);
    }
}

