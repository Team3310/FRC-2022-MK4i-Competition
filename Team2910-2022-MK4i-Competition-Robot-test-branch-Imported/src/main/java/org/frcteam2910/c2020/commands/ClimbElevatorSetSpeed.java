package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.ClimbElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbElevatorSetSpeed extends CommandBase {
    private final ClimbElevator elevator;
    private double speed;

    public ClimbElevatorSetSpeed(ClimbElevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        elevator.setHoldElevator();
        return true;
    }
}