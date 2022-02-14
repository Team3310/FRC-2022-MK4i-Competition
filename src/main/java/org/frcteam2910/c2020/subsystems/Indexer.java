package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import org.frcteam2910.c2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase {

    public enum IndexMode{
        MANUAL, MOTION_MAGIC
    }

    // Motor Controllers
    private TalonFX indexMotor;

    // Misc
    private IndexMode controlMode = IndexMode.MANUAL;
  

    //Conversions 
    private static final double ELEVATOR_OUTPUT_TO_ENCODER_RATIO = 0;
    private static final double INDEXER_DEGREES_TO_ENCODER_TICKS = 0;


    private final static Indexer INSTANCE = new Indexer();

    private Indexer() {
        indexMotor = new TalonFX(Constants.INDEX_MOTOR_ID);
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        indexMotor.configAllSettings(configs);

        indexMotor.setNeutralMode(NeutralMode.Brake);
        indexMotor.configMotionCruiseVelocity(6000);
        indexMotor.configMotionAcceleration(14000);
        indexMotor.configMotionSCurveStrength(4);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;
        statorCurrentConfigs.enable = false;
        indexMotor.configStatorCurrentLimit(statorCurrentConfigs);

        indexMotor.config_kF(Constants.INDEXER_MM_PORT, 0.0);
        indexMotor.config_kP(Constants.INDEXER_MM_PORT, 0.0);
        indexMotor.config_kI(Constants.INDEXER_MM_PORT, 0.0);
        indexMotor.config_kD(Constants.INDEXER_MM_PORT, 0.0);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    public double getIndexerRotations(){
        return indexMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / ELEVATOR_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getIndexerDegrees(){
        return getIndexerRotations() * 360;
    }

    public double getElevatorEncoderTicksAbsolute(double degrees){
        return (int) (degrees * INDEXER_DEGREES_TO_ENCODER_TICKS);
    }

    public synchronized void setIndexerMotionMagicPositionAbsolute(double degrees) {
        double targetPositionTicks;
        controlMode = IndexMode.MOTION_MAGIC;
        indexMotor.selectProfileSlot(1, 0);
        targetPositionTicks = getElevatorEncoderTicksAbsolute(degrees);
        indexMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public void setIndexerSpeed(double speed) {
        this.indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setIndexerRPM(double RPM){
        this.indexMotor.set(ControlMode.Velocity, RPM);
    }

    public synchronized void setHoldIndexer(){
        setIndexerMotionMagicPositionAbsolute(getIndexerDegrees());
    }
}
