package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.BalanceElevator;
import org.frcteam2910.c2020.subsystems.ClimbElevator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {
    
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final Intake intake = Intake.getInstance();
    private final ClimbElevator climbElevator = ClimbElevator.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final BalanceElevator balanceElevator = BalanceElevator.getInstance();
    private final Indexer indexer = Indexer.getInstance();

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    @SuppressWarnings("unused")
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
        intake.setController(secondaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        CommandScheduler.getInstance().setDefaultCommand(climbElevator, new ClimbControlJoysticks(climbElevator, getClimbElevatorAxis()));
        CommandScheduler.getInstance().setDefaultCommand(balanceElevator, new BalanceControlJoysticks(balanceElevator, getBalanceElevatorAxis()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        primaryController.getBackButton().whenPressed(
                new ZeroAll(balanceElevator, climbElevator, drivetrain)
        );
        primaryController.getStartButton().whenPressed(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
        );
        primaryController.getRightBumperButton().whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.ROBOT_CENTRIC)
        );
        primaryController.getRightBumperButton().whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
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
        secondaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
                new ClimbSetElevatorInches(climbElevator, 10)
        );

        //Indexer
        secondaryController.getRightBumperButton().whenPressed(
                new FeedBalls(intake, indexer, drivetrain, Constants.INDEXER_RPM)
        );
        secondaryController.getRightBumperButton().whenReleased(
                new IntakeIndexerHaltTeleOp(intake, indexer, drivetrain)
        );
        secondaryController.getLeftBumperButton().whenPressed(
                new EjectBalls(intake, indexer, shooter)
        );
        secondaryController.getLeftBumperButton().whenReleased(
                new IntakeIndexerHaltTeleOp(intake, indexer, drivetrain)
        );



        //Shooter
        secondaryController.getAButton().whenPressed(
                new ShooterShootWithHood(shooter, drivetrain, 1780, 11.0) //Fender
        );
        secondaryController.getBButton().whenPressed(
                new ShooterShootWithHood(shooter, drivetrain, 2030, 30.7) //RT Wall 24
        );
        secondaryController.getYButton().whenPressed(
                new ShooterShootWithHood(shooter, drivetrain, 2230, 36) //Hangar Shot
        );
        secondaryController.getXButton().whenPressed(
                new ShooterShootAllField(shooter, drivetrain)
        );
        secondaryController.getStartButton().whenPressed(
                new HoodAutoZero(shooter)
        );
       secondaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
                new ShooterShootAllFieldSearch(shooter, drivetrain, true)
        );
       secondaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(
                new ShooterShootAllFieldSearch(shooter, drivetrain, false)
       );


        //Misc
        secondaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
                new ExperimentalEjectBalls(intake, indexer)
        );
        secondaryController.getDPadButton(DPadButton.Direction.UP).whenReleased(
                new IntakeIndexerHalt(intake, indexer)
        );
        secondaryController.getDPadButton(DPadButton.Direction.DOWN).whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );



        SmartDashboard.putData("Auto Zero Hood", new HoodAutoZero(shooter));
        SmartDashboard.putData("Auto Zero Climb", new ClimbElevatorAutoZero(climbElevator));
        SmartDashboard.putData("Set Intake speed 0", new IntakeSetSpeed(intake, 0));
        SmartDashboard.putData("Set Intake speed 1", new IntakeSetSpeed(intake, 1.0));
        SmartDashboard.putData("Set Indexer speed 0", new IndexerSetSpeed(indexer, 0));

        SmartDashboard.putData("Set Shooter speed 0", new ShooterSetSpeed(shooter, 0));
        SmartDashboard.putData("Set Shooter RPM 2055", new ShooterSetRPM(shooter, 2055));


        SmartDashboard.putData("Set Hood 32", new HoodSetAngle(shooter, 32));
        SmartDashboard.putData("Set Hood 45", new HoodSetAngle(shooter, 45));
        SmartDashboard.putData("Set Hood 42", new HoodSetAngle(shooter, 42));
        SmartDashboard.putData("Set Hood 38", new HoodSetAngle(shooter, 38));

        SmartDashboard.putData("Distance offset -20", new InstantCommand(()-> shooter.setShooterDistanceOffset(-20)));
        SmartDashboard.putData("Distance offset -10", new InstantCommand(()-> shooter.setShooterDistanceOffset(-10)));
        SmartDashboard.putData("Distance offset -5", new InstantCommand(()-> shooter.setShooterDistanceOffset(-5)));
        SmartDashboard.putData("Distance offset 0", new InstantCommand(()-> shooter.setShooterDistanceOffset(0)));
        SmartDashboard.putData("Distance offset +5", new InstantCommand(()-> shooter.setShooterDistanceOffset(5)));
        SmartDashboard.putData("Distance offset +10", new InstantCommand(()-> shooter.setShooterDistanceOffset(10)));
        SmartDashboard.putData("Distance offset +20", new InstantCommand(()-> shooter.setShooterDistanceOffset(20)));


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

    public ClimbElevator getClimbElevator() {
        return climbElevator;
    }

    public Intake getIntakeSubsystem() {
        return intake;
    }
    public Shooter getShooter() {
        return shooter;
    }
    public Indexer getIndexer() {
        return indexer;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }
}
