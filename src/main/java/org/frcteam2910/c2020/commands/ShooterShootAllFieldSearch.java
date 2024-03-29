package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;

public class ShooterShootAllFieldSearch extends CommandBase {
    private final Shooter shooter;
    private final DrivetrainSubsystem drive;
    private  boolean isRight;

    public ShooterShootAllFieldSearch(Shooter shooter, DrivetrainSubsystem drive, boolean isRight) {
        this.shooter = shooter;
        this.drive = drive;
        this.isRight = isRight;
    }

    @Override
    public void initialize() {
        if(!shooter.hasTarget()){
            shooter.setHoodMotionMagicPositionAbsolute(30);
        }
        drive.setRotationRight(isRight);
        drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT_SEARCH);
    }

    @Override
    public void execute(){
        shooter.updateAllFieldShot(false);
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