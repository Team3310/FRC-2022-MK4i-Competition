package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2020.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase {

    public enum IndexerMode {
        MANUAL, POSITION, VELOCITY
    }

    // Motor Controllers
    private TalonFX indexMotor;

    // Misc
    private IndexerMode controlMode = IndexerMode.MANUAL;
    private final DigitalInput sensor;
  

    //Conversions 
    private static final double INDEXER_OUTPUT_TO_ENCODER_RATIO = 5.0;
    private static final double INDEXER_DEGREES_TO_ENCODER_TICKS = 360.0 / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private static final double INDEXER_REVOLUTIONS_TO_ENCODER_TICKS = INDEXER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private static final int INDEXER_POSITION_SLOT = 0;
    private static final int INDEXER_VELOCITY_SLOT = 1;

    private final static Indexer INSTANCE = new Indexer();

    private Indexer() {
        indexMotor = new TalonFX(Constants.INDEX_MOTOR_ID);
        sensor = new DigitalInput(Constants.INDEXER_DIO_PORT);
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        indexMotor.configAllSettings(configs);

        indexMotor.setInverted(TalonFXInvertType.Clockwise);
        indexMotor.setNeutralMode(NeutralMode.Brake);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 60;
        statorCurrentConfigs.enable = true;
        indexMotor.configStatorCurrentLimit(statorCurrentConfigs);

        indexMotor.config_kF(INDEXER_POSITION_SLOT, 0.0);
        indexMotor.config_kP(INDEXER_POSITION_SLOT, 0.2);
        indexMotor.config_kI(INDEXER_POSITION_SLOT, 0.0);
        indexMotor.config_kD(INDEXER_POSITION_SLOT, 0.0);

        indexMotor.config_kF(INDEXER_VELOCITY_SLOT, 0.0);
        indexMotor.config_kP(INDEXER_VELOCITY_SLOT, 0.1);
        indexMotor.config_kI(INDEXER_VELOCITY_SLOT, 0.0);
        indexMotor.config_kD(INDEXER_VELOCITY_SLOT, 0.0);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    public boolean getIndexerSensor(){
        return sensor.get();
    }

    public void setIndexerSpeed(double speed) {
        controlMode = IndexerMode.MANUAL;
        this.indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void setIndexerPositionAbsolute(double degrees) {
        controlMode = IndexerMode.POSITION;
        indexMotor.selectProfileSlot(INDEXER_POSITION_SLOT, 0);
        indexMotor.set(ControlMode.Position, getIndexerEncoderTicksAbsolute(degrees), DemandType.ArbitraryFeedForward, 0.0);
    }

    public void setIndexerRPM(double rpm){
        controlMode = IndexerMode.VELOCITY;
        indexMotor.selectProfileSlot(INDEXER_VELOCITY_SLOT, 0);
        this.indexMotor.set(ControlMode.Velocity, indexerRPMToNativeUnits(rpm));
    }

    public synchronized void setHoldIndexer(){
        setIndexerPositionAbsolute(getIndexerDegrees());
    }
    
    public double indexerRPMToNativeUnits(double rpm) {
        return rpm * INDEXER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public double getIndexerRotations(){
        return indexMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INDEXER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getIndexerDegrees(){
        return getIndexerRotations() * 360;
    }

    public double getIndexerEncoderTicksAbsolute(double degrees){
        return (int) (degrees * INDEXER_DEGREES_TO_ENCODER_TICKS);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Index sensor", getIndexerSensor());
    }
}

