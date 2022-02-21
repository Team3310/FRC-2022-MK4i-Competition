package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
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
    private final Indexer indexer = Indexer.getInstance();

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error building trajectories");
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);
        intake.setController(secondaryController);

        // primaryController.getLeftXAxis().setInverted(true);
        // primaryController.getRightXAxis().setInverted(true);

        

        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        //CommandScheduler.getInstance().setDefaultCommand(drivetrain, new DriveCommand(drivetrain, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        CommandScheduler.getInstance().setDefaultCommand(climbElevator, new ClimbControlJoysticks(climbElevator, getClimbElevatorAxis()));
        CommandScheduler.getInstance().setDefaultCommand(balanceElevator, new BalanceControlJoysticks(balanceElevator, getBalanceElevatorAxis()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        primaryController.getBackButton().whenPressed(
            new ZeroAll(balanceElevator, climbElevator, drivetrain)
        );
        primaryController.getRightBumperButton().whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.LIMELIGHT)
        );
        primaryController.getLeftBumperButton().whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        //Intake
        secondaryController.getRightTriggerAxis().getButton(0.5).whenPressed(
                new IndexerBallStop(indexer)
        );
        secondaryController.getRightTriggerAxis().getButton(0.5).whenReleased(
                new IndexerSetSpeed(indexer, 0)
        );


        //Climb
        secondaryController.getBackButton().whenPressed(
                new ClimbElevatorAutoZero(climbElevator)
        );

        //Indexer
        secondaryController.getRightBumperButton().whenPressed(
                new FeedBalls(intake, indexer)
        );
        secondaryController.getRightBumperButton().whenReleased(
                new IntakeIndexerHalt(intake, indexer)
        );
        secondaryController.getLeftBumperButton().whenPressed(
                new EjectBalls(intake, indexer, shooter)
        );
        secondaryController.getLeftBumperButton().whenReleased(
                new IntakeIndexerHalt(intake, indexer)
        );

        //Shooter
        secondaryController.getAButton().whenPressed(
                new ShooterShootWithHood(shooter, 2500, 3) //Fender
        );
        secondaryController.getBButton().whenPressed(
                new ShooterShootWithHood(shooter, 2300, 29) //RT Wall 24
        );
        secondaryController.getYButton().whenPressed(
                new ShooterShootWithHood(shooter, 3250, 38) //Terminal
        );

        //Misc
        secondaryController.getLeftJoystickButton().whenPressed(
                new ExperimentalEjectBalls(intake, indexer)
        );



    
        SmartDashboard.putData("Zero Climb Elevator", new InstantCommand(() ->climbElevator.setElevatorZero(0)));
        SmartDashboard.putData("Set Climb Elevator 20 inches", new InstantCommand(() ->climbElevator.setElevatorMotionMagicPositionAbsolute(20.0)));

        SmartDashboard.putData("Set Shooter 1500 RPM", new ShooterSetRPM(shooter, 1500));

        SmartDashboard.putData("RT Wall Shooter", new ShooterSetRPM(shooter, 2750));
        SmartDashboard.putData("RT Wall Hood", new HoodSetAngle(shooter, 24));

        SmartDashboard.putData("Fender Hood", new HoodSetAngle(shooter, 3));
        SmartDashboard.putData("Fender Shooter", new ShooterSetRPM(shooter, 2500));

        SmartDashboard.putData("Terminal Shooter", new ShooterSetRPM(shooter, 3250));
        SmartDashboard.putData("Terminal Hood", new HoodSetAngle(shooter, 38));

        SmartDashboard.putData("Set Indexer 0.7 speed", new IndexerSetSpeed(indexer, 0.7));

        SmartDashboard.putData("Zero Hood", new InstantCommand(() -> shooter.resetHoodHomePosition()));
        SmartDashboard.putData("Set Hood 10 deg", new HoodSetAngle(shooter, 10));

        SmartDashboard.putData("Indexer Stop", new IndexerBallStop(indexer));
        
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
