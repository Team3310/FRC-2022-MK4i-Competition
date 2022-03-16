package org.frcteam2910.c2020.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.util.Units;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Pigeon;
import org.frcteam2910.c2020.Robot;
import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.PlaystationController;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
    public static final double TRACKWIDTH = 0.502;
    public static final double WHEELBASE = 0.502;
    public static final double WHEEL_DIAMETER_INCHES = 4.00;  // Actual is 3.89"

    private double frontLeftOffset = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_COMP_OFFSET;
    private double frontRightOffset = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_COMP_OFFSET;
    private double backLeftOffset = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_COMP_OFFSET;
    private double backRightOffset = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_COMP_OFFSET;

    public double rCurrPoseX;
    public double rCurrPoseY;
    public double rDistToGoal;
    public double rTurretGoalAngle;

    public TrapezoidProfile.Constraints constraints = new Constraints(6.0, 6.0);

    public enum DriveControlMode{
        JOYSTICKS, LIMELIGHT, ROTATION, TRAJECTORY,
         ROBOT_CENTRIC
    }

    public ProfiledPIDController rotationController = new ProfiledPIDController(1.0, 0.03, 0.02, constraints, 0.02);
    public PIDController limelightController = new PIDController(3.0, 0.03, 0.02, 0.02);
    

    DriveControlMode driveControlMode = DriveControlMode.JOYSTICKS;
    //CANivore string key is "Drivetrain" which can be located in the ctre Falcon 500 factories

       public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
            0.042746,
            0.0032181,
            0.30764
    );

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(), FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(12.5 * 12.0),
            new CentripetalAccelerationConstraint(15 * 12.0)
    };

    private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.4, 0.0, 0.025),
            new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
    );

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         //front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        //front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),       //back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        //back right
    );

    private final SwerveModule[] modules;

    private final Object sensorLock = new Object();
    Limelight limelight = Limelight.getInstance();

    @GuardedBy("sensorLock")
    private final Gyroscope gyroscope = new Pigeon(Constants.PIGEON_PORT);


    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;
    @GuardedBy("kinematicsLock")
    private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
    @GuardedBy("kinematicsLock")
    private Vector2 velocity = Vector2.ZERO;
    @GuardedBy("kinematicsLock")
    private double angularVelocity = 0.0;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    private Controller primaryController;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;

    public DrivetrainSubsystem() {
        synchronized (sensorLock) {
            gyroscope.setInverted(false);
        }

        if(Robot.isPracticeBot()){
            frontLeftOffset = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET;
            frontRightOffset = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET;
            backLeftOffset = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET;
            backRightOffset = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET;
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withPosition(0, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                frontLeftOffset
        );
        SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withPosition(1, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                frontRightOffset
        );
        SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withPosition(2, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                backLeftOffset
        );
        SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withPosition(3, 0)
                        .withSize(1, 3),
                Mk4SwerveModuleHelper.GearRatio.L2i,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                backRightOffset
        );

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        tab.addNumber("Trajectory X", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().x;
                })
                .withPosition(1, 0)
                .withSize(1, 1);
        tab.addNumber("Trajectory Y", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().y;
                })
                .withPosition(1, 1)
                .withSize(1, 1);

        tab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                signal = driveSignal;
            }

            if (signal == null) {
                return 0.0;
            }

            return signal.getRotation() * RobotController.getBatteryVoltage();
        });

        for(SwerveModule i : modules){
            i.setVoltageRamp(0.45);
        }

        tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);
    }

    public void setCoast(){
        for(SwerveModule i : modules){
            i.setMotorNeutralMode(NeutralMode.Coast);
        }
    }

    public void setBrake(){
        for(SwerveModule i : modules){
            i.setMotorNeutralMode(NeutralMode.Brake);
        }
    }


    public void setController(XboxController controller){
        primaryController = controller;
    }
    public void setController(PlaystationController controller){
        primaryController = controller;
    }

    public void setDriveControlMode(DriveControlMode mode){
        driveControlMode = mode;
    }

    public DriveControlMode getDriveControlMode(){
        return driveControlMode;
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity*0.75, isFieldOriented);
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    public double getRobotToGoalAngle() {
        rCurrPoseX = getPose().translation.x;
        rCurrPoseY = getPose().translation.y;
        return Math.toDegrees(Math.atan2(-(rCurrPoseY + 162), rCurrPoseX - 325));
    }

    public double getRobotToGoalDistance() {
        rCurrPoseX = getPose().translation.x - 325;
        rCurrPoseY = getPose().translation.y + 162;
        return Math.sqrt(rCurrPoseX*rCurrPoseX + rCurrPoseY*rCurrPoseY);
    }

    public void joystickDrive(){
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), getDriveRotationAxis().get(true), true);
    }

    public void robotCentricDrive(){
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), getDriveRotationAxis().get(true), false);
    }

    public void setRotationTarget(double goal){
        rotationController.enableContinuousInput(0.0, Math.PI*2);
        rotationController.reset(getPose().rotation.toRadians());
        rotationController.setGoal(goal + getPose().rotation.toRadians());
        rotationController.setTolerance(0.087);
        setDriveControlMode(DriveControlMode.ROTATION);
    }

    public boolean atRotationTarget(){
        
        if(rotationController.atGoal()){
            System.out.println("Reached target");
        }
        return rotationController.atGoal();
    }

    public void rotationDrive(){
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = rotationController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    }

    public void setLimelightTarget(){
        limelightController.enableContinuousInput(0.0, Math.PI*2);
        setDriveControlMode(DriveControlMode.LIMELIGHT);
    }

    public void limelightDrive(){

        limelightController.setSetpoint(Math.toRadians(-limelight.getTargetHorizOffset()) + getPose().rotation.toRadians());

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = limelightController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    }

    public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        for (var module : modules) {
            averageVelocity += Math.abs(module.getDriveVelocity());
        }
        return averageVelocity / 4;
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle())).scale(module.getDriveVelocity() * 39.37008 * WHEEL_DIAMETER_INCHES / 4.0);
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            angularVelocity = gyroscope.getRate();
        }

        SmartDashboard.putNumber("Current Gyro Angle", angle.toDegrees());
        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {

            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);
            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = angularVelocity;
        }
    }

    /** Updates ChassisVelocity and calculates SwerveModule velocity outputs. */
    private void updateModules(HolonomicDriveSignal driveSignal) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
        }
    }

    public void resetSteerAbsoluteAngle() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].resetAbsoluteSteerAngle();
        }
    }

    public void alignWheels() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].set(0,0);
        }
    }

    // 500 iterations is the default, -1 turns off auto reset
    public void setSteerEncoderAutoResetIterations(int iterations) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setEncoderAutoResetIterations(iterations);
        }
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    /**
     * Must be implemented for <code>UpdateManager</code> (Updatable interface method)
     * @param time - Time in seconds from Timer.getFPGATimeStamp() (robot runtime timer).
     * @param dt - Time since update was last called
     */
    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);

        DriveControlMode i_controlMode = getDriveControlMode();

        HolonomicDriveSignal currentDriveSignal = null;

        switch(i_controlMode){
            case JOYSTICKS:
                joystickDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case ROTATION:
                rotationDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case ROBOT_CENTRIC:
                robotCentricDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT:
                limelightDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case TRAJECTORY:
                Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                        getPose(),
                        getVelocity(),
                        getAngularVelocity(),
                        time,
                        dt
                );
                if (trajectorySignal.isPresent()) {
                    currentDriveSignal = trajectorySignal.get();
                    currentDriveSignal = new HolonomicDriveSignal(
                        currentDriveSignal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                        currentDriveSignal.getRotation() / RobotController.getBatteryVoltage(),
                        currentDriveSignal.isFieldOriented()
                    );
                }
                break; 
        }

        updateModules(currentDriveSignal);
    }

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        SmartDashboard.putNumber("Angle to goal", getRobotToGoalAngle());
        SmartDashboard.putNumber("Distance to goal", getRobotToGoalDistance());
        SmartDashboard.putNumber("X", getPose().translation.x);
        SmartDashboard.putNumber("Y", getPose().translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }
}
