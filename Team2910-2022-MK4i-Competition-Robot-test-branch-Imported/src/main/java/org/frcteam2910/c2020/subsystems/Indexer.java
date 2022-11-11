package org.frcteam2910.c2020.subsystems;

//new imports
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.ecus.TalonFX;
// import com.ctre.phoenix.motorcontrol.*;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

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
    @SuppressWarnings("unused")
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
        indexMotor = new TalonFX(Constants.INDEX_MOTOR_ID, null);
        sensor = new DigitalInput(Constants.INDEXER_DIO_PORT);
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        // configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        // indexMotor.configAllSettings(configs);
        configs.
        

        // indexMotor.setInverted(TalonFXInvertType.Clockwise);
        indexMotor.getConfigurator().(NeutralMode.BRAKE); //can't find this config

        configs.MotorOutput.Inverted = true;

        // final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        // statorCurrentConfigs.currentLimit = 60;
        // statorCurrentConfigs.enable = true;
        //indexMotor.configStatorCurrentLimit(statorCurrentConfigs);

        

        // indexMotor.config_kF(INDEXER_POSITION_SLOT, 0.0);
        // indexMotor.config_kP(INDEXER_POSITION_SLOT, 0.2);
        // indexMotor.config_kI(INDEXER_POSITION_SLOT, 0.0);
        // indexMotor.config_kD(INDEXER_POSITION_SLOT, 0.0);

        // indexMotor.config_kF(INDEXER_VELOCITY_SLOT, 0.0);
        // indexMotor.config_kP(INDEXER_VELOCITY_SLOT, 0.1);
        // indexMotor.config_kI(INDEXER_VELOCITY_SLOT, 0.0);
        // indexMotor.config_kD(INDEXER_VELOCITY_SLOT, 0.0);

        //kF is replaced by Kv
        configs.Slot0.kV = 0.0;
        configs.Slot0.kP = 0.2;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        configs.Slot1.kV = 0.0;
        configs.Slot1.kP = 0.1;
        configs.Slot1.kI = 0.0;
        configs.Slot1.kD = 0.0;

        indexMotor.getConfigurator().apply(configs);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    public boolean getIndexerSensor(){
        return sensor.get();
    }


    public void setIndexerSpeed(double speed) {
        controlMode = IndexerMode.MANUAL;
        PercentSupplyVoltageRequest request = new PercentSupplyVoltageRequest(speed);
        this.indexMotor.setControl(request);
        //this.indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void setIndexerPositionAbsolute(double degrees) {
        controlMode = IndexerMode.POSITION;
        indexMotor.selectProfileSlot(INDEXER_POSITION_SLOT, 0);
        PositionPercentSupplyVoltageRequest request = new PositionPercentSupplyVoltageRequest(degrees, 0);
        this.indexMotor.setControl(request);
        //indexMotor.set(ControlMode.Position, getIndexerEncoderTicksAbsolute(degrees), DemandType.ArbitraryFeedForward, 0.0);
    }

    public void setIndexerRPM(double rpm){
        controlMode = IndexerMode.VELOCITY;
        //indexMotor.selectProfileSlot(INDEXER_VELOCITY_SLOT, 0);
        VelocityPercentSupplyVoltageRequest request = new VelocityPercentSupplyVoltageRequest(rpm, 0);
        this.indexMotor.setControl(request);
        //this.indexMotor.set(ControlMode.Velocity, indexerRPMToNativeUnits(rpm));
    }

    public synchronized void setHoldIndexer(){
        setIndexerPositionAbsolute(getIndexerDegrees());
    }
    
    public double indexerRPMToNativeUnits(double rpm) {
        return rpm * INDEXER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public double getIndexerRotations(){
        return indexMotor.getPosition().getValue() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INDEXER_OUTPUT_TO_ENCODER_RATIO;
        //return indexMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INDEXER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getIndexerDegrees(){
        return getIndexerRotations() * 360;
    }

    public double getIndexerEncoderTicksAbsolute(double degrees){
        return (int) (degrees * INDEXER_DEGREES_TO_ENCODER_TICKS);
    }

    public double getIndexerRPM() {
        return indexMotor.getVelocity().getValue() / INDEXER_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
        //return indexMotor.getSelectedSensorVelocity() / INDEXER_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Index sensor", getIndexerSensor());
//        SmartDashboard.putNumber("Indexer RPM", getIndexerRPM());
    }
}

