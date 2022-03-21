package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
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

    // Conversions
    private static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 40.0 / 10.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    // Motor Controllers
    private TalonFX intakeMotor;

    // Misc
    private static final int kIntakeVelocitySlot = 0;
    private Controller secondaryController;
    private boolean hasSetIntakeZero = true;
    private boolean sysStatus = false;

    private final static Intake INSTANCE = new Intake();

    private Intake() {
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, "Drivetrain");

        intakeMotor.configFactoryDefault();

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        intakeMotor.configAllSettings(configs);

        intakeMotor.setInverted(TalonFXInvertType.CounterClockwise);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;
        statorCurrentConfigs.enable = false;
        intakeMotor.configStatorCurrentLimit(statorCurrentConfigs);

        intakeMotor.config_kF(kIntakeVelocitySlot, 0.055);
        intakeMotor.config_kP(kIntakeVelocitySlot, 0.10);
        intakeMotor.config_kI(kIntakeVelocitySlot, 0.0001);
        intakeMotor.config_kD(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.RollerRPMToNativeUnits(200));
    }

    public static Intake getInstance() {
        return INSTANCE;
    }

    public void setSystemStatus(boolean status) {
        sysStatus = status;
    }
    public boolean getSystemStatus(){
        return sysStatus;
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


        if(getRightTriggerAxis().getButton(0.1).get()){
            setRollerRPM(getRightTriggerAxis().get(true) * Constants.INTAKE_COLLECT_RPM);
            hasSetIntakeZero = false;
        }
        else if(getLeftTriggerAxis().getButton(0.1).get()){
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

    @Override
    public void periodic(){
        variableIntakeRPM();
        SmartDashboard.putNumber("Intake Amperage", intakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());
    }

//        SmartDashboard.putNumber("Intake Roller Rotations", this.getRollerRotations());
//        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());

}

