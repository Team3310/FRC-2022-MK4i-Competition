package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Robot;
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
		elevator.setZeroing(true);
		lastElevatorPosition = Constants.ELEVATOR_MAX_INCHES;
		elevator.setElevatorSpeed(Constants.ELEVATOR_AUTO_ZERO_SPEED);
		encoderCount = 0;
//		System.out.println("Auto zero initialize");
    }

    @Override
    public boolean isFinished() {
		elevator.setElevatorSpeedZeroing(Constants.ELEVATOR_AUTO_ZERO_SPEED);
		double currentElevatorPosition = elevator.getElevatorInches();
		double elevatorPositionChange = lastElevatorPosition - currentElevatorPosition;
		lastElevatorPosition = currentElevatorPosition;
		boolean test = encoderCount > 2 && Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE && elevator.getElevatorMotorCurrent() > elevator.AUTO_ZERO_MOTOR_CURRENT;
		//System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", elevator change = " + elevatorPositionChange + ", current = " + elevator.getElevatorMotorCurrent());
		
		if (Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE) {
			encoderCount++;
		}
		else {
			encoderCount = 0;
		}
		
		return test;
    }

	@Override
	public void end(boolean interrupted){
		elevator.setElevatorSpeedZeroing(0);
		elevator.setElevatorZero(Constants.ELEVATOR_HOME_POSITION);
		elevator.setZeroing(false);
		elevator.setHoldElevator();
	}
}