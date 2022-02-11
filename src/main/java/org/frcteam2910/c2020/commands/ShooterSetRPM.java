package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSetRPM extends CommandBase {
    private final Shooter shooter;
    private double RPM;

    public ShooterSetRPM(Shooter shooter, double RPM) {
        this.shooter = shooter;
        this.RPM = RPM;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(RPM);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}