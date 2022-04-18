package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.Shooter;

public class HoodAutoZero extends CommandBase {
    private final Shooter shooter;
    private double MIN_ELEVATOR_POSITION_CHANGE = 0.05;
	private double lastHoodPosition;
	private int encoderCount;

    public HoodAutoZero(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
		lastHoodPosition = Constants.HOOD_MAX_ANGLE_DEGREES;
		shooter.setHoodSpeed(Constants.HOOD_ZERO_SPEED);
		encoderCount = 0;
    }

    @Override
    public boolean isFinished() {
		shooter.setHoodSpeed(Constants.HOOD_ZERO_SPEED);
		double currentHoodPosition = shooter.getHoodAngleAbsoluteDegrees();
		double elevatorPositionChange = lastHoodPosition - currentHoodPosition;
		lastHoodPosition = currentHoodPosition;
		boolean test = encoderCount > 2 && Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE && shooter.getHoodCurrent() > shooter.AUTO_ZERO_MOTOR_CURRENT;
		//System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", elevator change = " + elevatorPositionChange + ", current = " + shooter.getHoodCurrent());
		
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
		shooter.setHoodSpeed(0);
		shooter.resetHoodHomePosition();
	}
}