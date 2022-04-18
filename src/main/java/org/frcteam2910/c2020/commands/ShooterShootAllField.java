package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.util.InterpolatingDouble;

public class ShooterShootAllField extends CommandBase {
    private final Shooter shooter;
    private final DrivetrainSubsystem drive;

    public ShooterShootAllField(Shooter shooter, DrivetrainSubsystem drive) {
        this.shooter = shooter;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        if(!shooter.hasTarget()){
            shooter.setHoodMotionMagicPositionAbsolute(30);
            drive.setTurnToTarget();
        }
        else if(shooter.hasTarget() && Limelight.getInstance().getFilteredTargetHorizOffset() > 3.0){
            drive.setTurnToTarget();
        }
        else{
            drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT);
        }
    }

    @Override
    public void execute(){
        shooter.updateAllFieldShot(false);
    }

    @Override
    public boolean isFinished() {
        return drive.getLimelightOverride() || drive.getDriveControlMode() == DrivetrainSubsystem.DriveControlMode.JOYSTICKS;
    }

    @Override
    public void end(boolean interrupted){
        shooter.setShooterRPM(Constants.IDLE_SHOOTER_RPM);
        shooter.setHoodMotionMagicPositionAbsolute(Constants.IDLE_HOOD_ANGLE);
    }
}