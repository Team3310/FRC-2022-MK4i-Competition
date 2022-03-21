package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Shooter;

public class ShooterShootWithHoodAuton extends CommandBase {
    private final Shooter shooter;
    private double angle;
    private double RPM;

    public ShooterShootWithHoodAuton(Shooter shooter, double RPM, double hoodAngle) {
        this.shooter = shooter;
        this.angle = hoodAngle;
        this.RPM = RPM;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodMotionMagicPositionAbsolute(angle);
        shooter.setShooterRPM(RPM);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}