package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;

public class ShooterShootAllFieldAuto extends CommandBase {
    private final Shooter shooter;

    public ShooterShootAllFieldAuto(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        if(!shooter.hasTarget()){
            shooter.setHoodMotionMagicPositionAbsolute(30);
        }
        shooter.startTimer();
    }

    @Override
    public void execute(){
        shooter.updateAllFieldShot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        shooter.resetTimer();
    }
}