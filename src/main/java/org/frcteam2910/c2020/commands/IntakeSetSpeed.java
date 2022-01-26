package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeSetSpeed extends CommandBase {
    private final Intake intake;
    private double speed;

    public IntakeSetSpeed(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setRollerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
