package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;

public class IntakeLiftAutoZero extends CommandBase {
    private final Intake intake;
    private double MIN_ELEVATOR_POSITION_CHANGE = 0.05;
	private double lastLiftPosition;
	private int encoderCount;

    public IntakeLiftAutoZero(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
		lastLiftPosition = Constants.LIFT_MAX_ANGLE_DEGREES;
		intake.setLiftSpeed(Constants.LIFT_ZERO_SPEED);
		encoderCount = 0;
    }

    @Override
    public boolean isFinished() {
		intake.setLiftSpeed(Constants.HOOD_ZERO_SPEED);
		double currentLiftPosition = intake.getLiftAngleAbsoluteDegrees();
		double elevatorPositionChange = lastLiftPosition - currentLiftPosition;
		lastLiftPosition = currentLiftPosition;
		boolean test = encoderCount > 2 && Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE && intake.getLiftCurrent() > intake.AUTO_ZERO_MOTOR_CURRENT;
		System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", elevator change = " + elevatorPositionChange + ", current = " + intake.getLiftCurrent());
		
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
		intake.setLiftSpeed(0);
		intake.resetLiftHomePosition();
	}
}