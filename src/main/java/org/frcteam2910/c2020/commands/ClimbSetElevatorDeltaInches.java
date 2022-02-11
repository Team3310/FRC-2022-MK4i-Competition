package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.ClimbElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbSetElevatorDeltaInches extends CommandBase {
    private final ClimbElevator elevator;
    private double deltaInches;
    double inches;

    public ClimbSetElevatorDeltaInches(ClimbElevator elevator, double deltaInches) {
        this.elevator = elevator;
        this.deltaInches = deltaInches;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        inches = elevator.getElevatorInches() + deltaInches;
        elevator.setElevatorMotionMagicPositionAbsolute(inches);
    }

    @Override
    public boolean isFinished() {
        elevator.setHoldElevator();
        return true;
    }
}