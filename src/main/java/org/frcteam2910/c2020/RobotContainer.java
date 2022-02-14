package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.ClimbControlJoysticks;
import org.frcteam2910.c2020.commands.ClimbElevatorSetSpeed;
import org.frcteam2910.c2020.commands.DriveCommand;
import org.frcteam2910.c2020.commands.HoodSetAngle;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.IntakeSetSpeed;
import org.frcteam2910.c2020.commands.ResetOdometryHeading;
import org.frcteam2910.c2020.commands.ShooterSetRPM;
import org.frcteam2910.c2020.subsystems.BalanceElevator;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;

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

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        

        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        CommandScheduler.getInstance().setDefaultCommand(drivetrain, new DriveCommand(drivetrain, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        CommandScheduler.getInstance().setDefaultCommand(climbElevator, new ClimbControlJoysticks(climbElevator, getClimbElevatorAxis()));

        driverReadout = new DriverReadout(this);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
		primaryController.getBackButton().whenPressed(
            () -> drivetrain.resetGyroAngle(Rotation2.ZERO)
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
    
        SmartDashboard.putData("Zero Climb Elevator", new InstantCommand(() ->climbElevator.setElevatorZero()));
        SmartDashboard.putData("Set Climb Elevator 20 inches", new InstantCommand(() ->climbElevator.setElevatorMotionMagicPositionAbsolute(20.0)));

        SmartDashboard.putData("Set Shooter 1500 RPM", new ShooterSetRPM(shooter, 1500));

        SmartDashboard.putData("Zero Hood", new InstantCommand(() -> shooter.resetHoodHomePosition()));
        SmartDashboard.putData("Set Hood 30 deg", new HoodSetAngle(shooter, 30));
        
        
        SmartDashboard.putData("Zero Balance Elevator", new InstantCommand(() ->balanceElevator.setElevatorZero()));
        SmartDashboard.putData("Set Balance Elevator 20 inches", new InstantCommand(() ->balanceElevator.setBalanceElevatorMotionMagicPositionAbsolute(20.0)));
 
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    private Axis getClimbElevatorAxis() {
        return secondaryController.getLeftYAxis();
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
