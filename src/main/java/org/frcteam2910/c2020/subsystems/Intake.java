package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.util.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    // Conversions
    private static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 30.0 / 12.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    // Motor Controllers
    private TalonFX intakeMotor;

    // Misc
    private static final int kIntakeVelocitySlot = 0;
    private double targetPositionTicks = 0;

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

    public double RollerRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

//        SmartDashboard.putNumber("Intake Roller Rotations", this.getRollerRotations());
//        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());

}

