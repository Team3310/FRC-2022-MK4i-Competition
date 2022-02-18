package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.ActivateZeroMode;
import org.frcteam2910.c2020.commands.BalanceControlJoysticks;
import org.frcteam2910.c2020.commands.ClimbControlJoysticks;
import org.frcteam2910.c2020.commands.ClimbElevatorSetSpeed;
import org.frcteam2910.c2020.commands.ClimbSetElevatorInches;
import org.frcteam2910.c2020.commands.ClimbStageOne;
import org.frcteam2910.c2020.commands.DriveCommand;
import org.frcteam2910.c2020.commands.HoodSetAngle;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.IntakeSetSpeed;
import org.frcteam2910.c2020.commands.ShooterSetRPM;
import org.frcteam2910.c2020.commands.ZeroAll;
import org.frcteam2910.c2020.subsystems.BalanceElevator;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.subsystems.BalanceElevator.BalanceControlMode;
import org.frcteam2910.c2020.subsystems.ClimbElevator.ClimbControlMode;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.robot.input.DPadButton.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {
    
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final Intake intake = Intake.getInstance();
    private final ClimbElevator climbElevator = ClimbElevator.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final BalanceElevator balanceElevator = BalanceElevator.getInstance();

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    private final DriverReadout driverReadout;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error building trajectories");
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);

        // primaryController.getLeftXAxis().setInverted(true);
        // primaryController.getRightXAxis().setInverted(true);

        

        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        //CommandScheduler.getInstance().setDefaultCommand(drivetrain, new DriveCommand(drivetrain, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        CommandScheduler.getInstance().setDefaultCommand(climbElevator, new ClimbControlJoysticks(climbElevator, getClimbElevatorAxis()));
        CommandScheduler.getInstance().setDefaultCommand(balanceElevator, new BalanceControlJoysticks(balanceElevator, getBalanceElevatorAxis()));
        driverReadout = new DriverReadout(this);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
		primaryController.getBackButton().whenPressed(
            new ZeroAll(balanceElevator, climbElevator, drivetrain)
        );

		// ResetOdometryHeading sometimes gets the robot stuck (won't respond to drive commands)
        //primaryController.getBackButton().whenPressed(new ResetOdometryHeading(drivetrainSubsystem));

        primaryController.getRightBumperButton().whenPressed(
            new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM)
            //new IntakeSetSpeed(intakeSubsystem, 0.5)
        );
        primaryController.getRightBumperButton().whenReleased(
            new IntakeSetSpeed(intake, 0.0)
        );
        primaryController.getLeftBumperButton().whenPressed(
            new IntakeSetRPM(intake, Constants.INTAKE_REVERSE_RPM)
        );
        primaryController.getLeftBumperButton().whenReleased(
            new IntakeSetSpeed(intake, 0.0)
        ); 
        secondaryController.getDPadButton(Direction.DOWN).whenPressed(
            new ClimbStageOne(shooter, balanceElevator, climbElevator)
        ); 
        secondaryController.getDPadButton(Direction.UP).whenPressed(
            new ClimbSetElevatorInches(climbElevator, Constants.ELEVATOR_MAX_INCHES)
        );
        secondaryController.getDPadButton(Direction.RIGHT).whenPressed(
            () -> climbElevator.setControlMode(ClimbControlMode.ZERO)
        );
        secondaryController.getDPadButton(Direction.LEFT).whenPressed(
            () -> climbElevator.setControlMode(ClimbControlMode.MANUAL)
        );
    
        SmartDashboard.putData("Zero Climb Elevator", new InstantCommand(() ->climbElevator.setElevatorZero()));
        SmartDashboard.putData("Set Climb Elevator 20 inches", new InstantCommand(() ->climbElevator.setElevatorMotionMagicPositionAbsolute(20.0)));

        SmartDashboard.putData("Set Shooter 1500 RPM", new ShooterSetRPM(shooter, 1500));

        SmartDashboard.putData("Zero Hood", new InstantCommand(() -> shooter.resetHoodHomePosition()));
        SmartDashboard.putData("Set Hood 10 deg", new HoodSetAngle(shooter, 10));
        
        
        SmartDashboard.putData("Zero Balance Elevator", new InstantCommand(() ->balanceElevator.setElevatorZero()));
        SmartDashboard.putData("Set Balance Elevator 10 inches", new InstantCommand(() ->balanceElevator.setBalanceElevatorMotionMagicPositionAbsolute(10.0)));
 
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getClimbElevatorAxis() {
        return secondaryController.getLeftYAxis();
    }

    private Axis getBalanceElevatorAxis(){
        return secondaryController.getRightYAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrain;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }
}
