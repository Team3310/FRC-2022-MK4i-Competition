package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSetSpeed extends CommandBase {
    private final Shooter shooter;
    private double speed;

    public ShooterSetSpeed(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
