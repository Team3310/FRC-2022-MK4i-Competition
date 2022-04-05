package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;


public class IntakeLiftSetAngle extends CommandBase {
    private final Intake intake;
    private double angle;

    public IntakeLiftSetAngle(Intake intake, double angle) {
        this.intake = intake;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        //intake.setLiftMotionMagicPositionAbsolute(angle);
        intake.setLiftPIDPositionAbsoluteInternal(angle);
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}
