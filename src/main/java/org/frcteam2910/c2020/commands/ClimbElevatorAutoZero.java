package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.subsystems.ClimbElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbElevatorAutoZero extends CommandBase {
    private final ClimbElevator elevator;
    private double MIN_ELEVATOR_POSITION_CHANGE = 0.05;
	private double lastElevatorPosition;
	private int encoderCount;

    public ClimbElevatorAutoZero(ClimbElevator elevator) {
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
		lastElevatorPosition = Constants.MAX_POSITION_INCHES;
		elevator.setSpeed(Constants.AUTO_ZERO_SPEED);
		encoderCount = 0;
//		System.out.println("Auto zero initialize");
    }

    @Override
    public boolean isFinished() {
		elevator.setSpeed(Constants.AUTO_ZERO_SPEED);
		double currentElevatorPosition = elevator.getPositionInches();
		double elevatorPositionChange = lastElevatorPosition - currentElevatorPosition;
		lastElevatorPosition = currentElevatorPosition;
		boolean test = encoderCount > 2 && Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE && Robot.elevator.getAverageMotorCurrent() > Elevator.AUTO_ZERO_MOTOR_CURRENT;
		System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", elevator change = " + elevatorPositionChange + ", current = " + Robot.elevator.getAverageMotorCurrent());
		
		if (Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE) {
			encoderCount++;
		}
		else {
			encoderCount = 0;
		}
		
		return test;
    }
}