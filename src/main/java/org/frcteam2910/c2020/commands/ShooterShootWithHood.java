package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;

public class ShooterShootWithHood extends CommandBase {
    private final Shooter shooter;
    private final DrivetrainSubsystem drive;
    private double angle;
    private double RPM;

    public ShooterShootWithHood(Shooter shooter, DrivetrainSubsystem drive, double RPM, double hoodAngle) {
        this.shooter = shooter;
        this.drive = drive;
        this.angle = hoodAngle;
        this.RPM = RPM;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        //drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT);
        shooter.setHoodMotionMagicPositionAbsolute(angle);
        shooter.setShooterRPM(RPM);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}