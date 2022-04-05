package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;

public class ShooterShootWithHood extends CommandBase {
    private final Shooter shooter;
    private final DrivetrainSubsystem drive;
    private double angle;
    private double RPM;
    private DrivetrainSubsystem.SwervePivotPoint pivotPoint;

    public ShooterShootWithHood(Shooter shooter, DrivetrainSubsystem drive, double RPM, double hoodAngle) {
        this(shooter, DrivetrainSubsystem.SwervePivotPoint.CENTER, drive, RPM, hoodAngle);
    }

    public ShooterShootWithHood(Shooter shooter, DrivetrainSubsystem.SwervePivotPoint pivotPoint, DrivetrainSubsystem drive, double RPM, double hoodAngle) {
        this.shooter = shooter;
        this.drive = drive;
        this.angle = hoodAngle;
        this.RPM = RPM;
        this.pivotPoint = pivotPoint;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT);
        drive.setSwervePivotPoint(pivotPoint);
        shooter.setHoodMotionMagicPositionAbsolute(angle);
        shooter.setShooterRPM(RPM);
    }

    @Override
    public boolean isFinished() {
        return drive.getDriveControlMode() == DrivetrainSubsystem.DriveControlMode.JOYSTICKS;
    }

    @Override
    public void end(boolean interrupted){
        shooter.setShooterRPM(Constants.IDLE_SHOOTER_RPM);
        shooter.setHoodMotionMagicPositionAbsolute(Constants.IDLE_HOOD_ANGLE);
    }
}