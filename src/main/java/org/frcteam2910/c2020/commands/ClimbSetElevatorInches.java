package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.ClimbElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbSetElevatorInches extends CommandBase {
    private final ClimbElevator elevator;
    private double inches;

    public ClimbSetElevatorInches(ClimbElevator elevator, double inches) {
        this.elevator = elevator;
        this.inches = inches;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorMotionMagicPositionAbsolute(inches);
    }

    @Override
    public boolean isFinished() {
        //elevator.setHoldElevator();
        return true;
    }
}