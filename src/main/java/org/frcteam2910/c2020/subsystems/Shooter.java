package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.util.InterpolatingDouble;


public class Shooter extends SubsystemBase {

    public static final double AUTO_ZERO_MOTOR_CURRENT = 1.0;

    public enum HoodControlMode {
        MANUAL, MOTION_MAGIC,
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
    private double distanceOffset = -5.0; //-5
    Limelight limelight = Limelight.getInstance();
    private InterpolatingDouble RPM = new InterpolatingDouble(0.0);
    private InterpolatingDouble hoodAngle = new InterpolatingDouble(0.0);
    private  InterpolatingDouble actualDist = new InterpolatingDouble(0.0);
    private  double limeLightDistance;
    private boolean isShooting = false;
    private boolean isPoseUpdated = false;
    private double timeStarted = 0;
    private double timeToWindUp = 0;
    private boolean timerEnded = false;

    private final static Shooter INSTANCE = new Shooter();

    private Shooter() {
        shooterMotorMaster = new TalonFX(Constants.SHOOTER_MASTER_CAN_ID);
        shooterMotorSlave = new TalonFX(Constants.SHOOTER_SLAVE_CAN_ID);
        hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        configs.voltageCompSaturation = 12.0;
        shooterMotorMaster.configAllSettings(configs);
        shooterMotorSlave.configAllSettings(configs);
        hoodMotor.configAllSettings(configs);

        shooterMotorMaster.enableVoltageCompensation(true);
        shooterMotorSlave.enableVoltageCompensation(true);

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
        statorCurrentConfigs.enable = false;
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

        shooterMotorMaster.config_kF(0, 0.049);
        shooterMotorMaster.config_kP(0, 0.04); //.01
        shooterMotorMaster.config_kI(0, 0.000005);
        shooterMotorMaster.config_IntegralZone(0, this.shooterRPMToNativeUnits(100));
        shooterMotorMaster.config_kD(0, 4.0);

        hoodMotor.config_kF(Constants.HOOD_MM_PORT, 0.045);
        hoodMotor.config_kP(Constants.HOOD_MM_PORT, 0.5);//.9
        hoodMotor.config_kI(Constants.HOOD_MM_PORT, 0.008);//.008
        hoodMotor.config_kD(Constants.HOOD_MM_PORT, 0.0);

        hoodMotor.config_kF(Constants.HOOD_PID_PORT, 0.045);
        hoodMotor.config_kP(Constants.HOOD_PID_PORT, 0.25);//.9
        hoodMotor.config_kI(Constants.HOOD_PID_PORT, 0.008);//.008
        hoodMotor.config_kD(Constants.HOOD_PID_PORT, 0.0);
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    public void setIsShooting(boolean isShooting){
        if (this.isShooting == false && isShooting == true) {
            this.isPoseUpdated = false;
        }
        this.isShooting = isShooting;
    }

    public boolean isShooting(){
        return isShooting;
    }

    public void startTimer() {
        timeStarted = Timer.getFPGATimestamp();
    }

    public void endTimer(){
        timeToWindUp = Timer.getFPGATimestamp() - timeStarted;
        timerEnded = true;
    }

    public void resetTimer(){
        timerEnded = false;
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

    public void setShooterDistanceOffset(double offset){
        this.distanceOffset = offset;
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

    public void setHoodSpeed(double speed){
        hoodMotor.set(ControlMode.PercentOutput, speed);
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
        hoodMotor.setSelectedSensorPosition(0);
    }

    public double getHoodAngleAbsoluteDegrees() {
        return (double)hoodMotor.getSelectedSensorPosition() / HOOD_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees;
    }

    public double getHoodCurrent(){
        return hoodMotor.getStatorCurrent();
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
        hoodMotor.selectProfileSlot(Constants.HOOD_PID_PORT, 0);
        double limitedAngle = limitHoodAngle(angle);
        targetPositionTicks = getHoodEncoderTicksAbsolute(limitedAngle);
        hoodMotor.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.07);
    }

    public synchronized boolean hasFinishedHoodTrajectory() {
        return hoodControlMode == HoodControlMode.MOTION_MAGIC
                && Util.epsilonEquals(hoodMotor.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    public double getLimelightDistanceFromGoal(){
        double distance;
        double heightLimelight = 30.56 + (13.1 * Math.sin(Math.toRadians(5.0 + getHoodAngleAbsoluteDegrees())));
        double heightOfGoal = 102.60;

        distance = (heightOfGoal - heightLimelight) / Math.tan(Math.toRadians((65.0 - getHoodAngleAbsoluteDegrees()) + limelight.getFilteredTargetVertOffset()));

        return distance + distanceOffset;
    }

    public double getBallVelocity(){
        double mainShooterSurfaceVel = (RPM.value/60.0) * Math.PI * 4.0;
        double kickerShooterSurfaceVel = mainShooterSurfaceVel*0.5*28.0/24.0;

        return (mainShooterSurfaceVel + kickerShooterSurfaceVel) / 2.0;
    }

    public double getTrajectoryAngleDegrees(){
        return 90.0 - hoodAngle.value;
    }

    public double getHorizontalBallVelocity(){
        return  getBallVelocity() * Math.cos(Math.toRadians(getTrajectoryAngleDegrees()));
    }

    public double getVerticalBallVelocity(){
        return  getBallVelocity() * Math.sin(Math.toRadians(getTrajectoryAngleDegrees()));
    }

    public double getMovingHoodAngleDegrees(){
        double angle = Math.atan2(getHorizontalBallVelocity() + DrivetrainSubsystem.getInstance().getVelocity().x, getVerticalBallVelocity());
        return Math.toDegrees(angle);
    }

    public double getMovingRPM(){
        double ballVelocity = getHorizontalBallVelocity()/Math.sin(Math.toRadians(getMovingHoodAngleDegrees()));
        double shooterMainVelocity = ballVelocity * 2.0 / (1.0 + 0.5 * 28.0 / 24.0);
        return shooterMainVelocity * 60.0 / (Math.PI * 4.0);
    }

    public double getBallFlightTime(){
        double vx = getHorizontalBallVelocity();
        if(vx > 0.1){
            return actualDist.value /getHorizontalBallVelocity();
        }
        else{
            return 1.0;
        }
    }

    public double getActualDistanceFromGoal(){
        return actualDist.value;
    }

    public double getMovingDistance(double timeOfFlight){
        double vx = DrivetrainSubsystem.getInstance().getVelocity().x;
        double virtualDist = vx * getBallFlightTime();
        double movingDist = limeLightDistance - virtualDist;
        return movingDist;
    }


    public boolean hasTarget(){
        return limelight.hasTarget();
    }



    public void updateAllFieldShot(){
        if(limelight.hasTarget()) {
            // While shooting freeze rpm and hood angle adjustments and update pose
            if (isShooting && !isPoseUpdated) {
                RigidTransform2 newPose = getPoseBasedOnLimelightAndGyro();
                if (newPose != RigidTransform2.ZERO) {
                    DrivetrainSubsystem.getInstance().resetPose(newPose);
                    isPoseUpdated = true;
                }
            }

            // Update RPM and hood angle based on limelight distance
            else {
                limeLightDistance = getLimelightDistanceFromGoal();

                RPM = Constants.kLobRPMMap.getInterpolated(new InterpolatingDouble(limeLightDistance));
                hoodAngle = Constants.kLobHoodMap.getInterpolated(new InterpolatingDouble(limeLightDistance));
                actualDist = Constants.kLimelightDistanceMap.getInterpolated(new InterpolatingDouble(limeLightDistance));

    //            setShooterRPM(getMovingRPM());
    //            setHoodMotionMagicPositionAbsolute(getMovingHoodAngleDegrees());

                setShooterRPM(RPM.value);
                setHoodMotionMagicPositionAbsolute(hoodAngle.value);
            }
        }
    }
    
    public RigidTransform2 getPoseBasedOnLimelightAndGyro() {
        if (limelight.getFilteredTargetHorizOffset() < 3.0) {
            Rotation2 gyroRotation = DrivetrainSubsystem.getInstance().getPose().rotation;
            double x = actualDist.value * gyroRotation.cos + 325.0;
            double y = actualDist.value * gyroRotation.sin - 162.0;
            return new RigidTransform2(new Vector2(x, y), gyroRotation);
        }

        return RigidTransform2.ZERO;
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Distance from goal rim", getLimelightDistanceFromGoal());
//        SmartDashboard.putNumber("Hood Angle", getHoodAngleAbsoluteDegrees());
//        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
//        SmartDashboard.putBoolean("Hood Reset", hoodReset);
//        SmartDashboard.putNumber("Current Offset", distanceOffset);
//        SmartDashboard.putNumber("Commanded RPM", commandedRPM);
//        SmartDashboard.putNumber("Moving RPM", getMovingRPM());
//        SmartDashboard.putNumber("Moving hood angle", getMovingHoodAngleDegrees());
//        SmartDashboard.putNumber("Limelight hood angle", hoodAngle.value);
//        SmartDashboard.putNumber("Limelight RPM", RPM.value);
        if((getShooterRPM() == RPM.value + 30.0 || getShooterRPM() == RPM.value - 30.0) && timerEnded){
            endTimer();
        }
        //System.out.println("Shooter wind up time: " + timeToWindUp);
    }
}

