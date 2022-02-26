package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.common.util.InterpolatingDouble;

public class AllFieldAuton extends CommandBase {
    private final Shooter shooter;
    private final DrivetrainSubsystem drive;

    public AllFieldAuton(Shooter shooter, DrivetrainSubsystem drive) {
        this.shooter = shooter;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        if(!shooter.hasTarget()){
            shooter.setHoodMotionMagicPositionAbsolute(30);
        }
        drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT);
    }

    @Override
    public void execute(){
        shooter.updateAllFieldShot();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted){
        shooter.setShooterRPM(Constants.IDLE_SHOOTER_RPM);
        shooter.setHoodMotionMagicPositionAbsolute(Constants.IDLE_HOOD_ANGLE);
    }
}