package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class HoodSetAngle extends CommandBase {
    private final Shooter shooter;
    private double angle;

    public HoodSetAngle(Shooter shooter, double angle) {
        this.shooter = shooter;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        shooter.setHoodMotionMagicPositionAbsolute(angle);
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}
