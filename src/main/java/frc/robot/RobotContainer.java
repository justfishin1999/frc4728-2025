// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GoHome;
import frc.robot.commands.GoHomeL1Only;
import frc.robot.commands.GoToL1;
import frc.robot.commands.GoToL2;
import frc.robot.commands.GoToL3;
import frc.robot.commands.GoToL4;
import frc.robot.commands.RunAlgae1;
import frc.robot.commands.RunAlgae2;
import frc.robot.commands.RunClimberIn;
import frc.robot.commands.RunClimberOut;
import frc.robot.commands.RunIntakeOut;
import frc.robot.commands.ScoreL1;
import frc.robot.commands.ScoreL2_3;
import frc.robot.commands.ScoreL4;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    private Elevator elevator =  new Elevator();
    private Intake intake = new Intake();
    private Wrist wrist = new Wrist();
    private Climber climber = new Climber();

    private double translationMultiplier, strafeMultiplier, rotateMultiplier;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick buttonBoard = new CommandJoystick(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //create auto chooser in dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        //register commands for path planner
        NamedCommands.registerCommand("ScoreL2_3",new ScoreL2_3(elevator, wrist, intake));
        NamedCommands.registerCommand("ScoreL1",new ScoreL1(elevator, wrist, intake));
        NamedCommands.registerCommand("ScoreL4",new ScoreL4(wrist, elevator));
        NamedCommands.registerCommand("GoHome",new GoHome(elevator, wrist));
        NamedCommands.registerCommand("GoHomeL1",new GoHomeL1Only(wrist, elevator));
        NamedCommands.registerCommand("GoGoL1",new GoToL1(elevator, wrist));
        NamedCommands.registerCommand("GoGoL2",new GoToL2(elevator, wrist));
        NamedCommands.registerCommand("GoGoL3",new GoToL3(elevator, wrist));
        NamedCommands.registerCommand("GoGoL4",new GoToL3(elevator, wrist));
        NamedCommands.registerCommand("RunAlgae1",new RunAlgae1(elevator, wrist, intake));
        NamedCommands.registerCommand("RunAlgae2",new RunAlgae1(elevator, wrist, intake));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.65*-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(0.65*-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(0.9*-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // reset the field-centric heading on left bumper press
        drivetrain.registerTelemetry(logger::telemeterize);

        //operator controls on button board
        buttonBoard.button(10).whileTrue(new RunClimberOut(climber));
        buttonBoard.button(9).whileTrue(new RunClimberIn(climber));
        buttonBoard.button(1).onTrue(new GoHome(elevator, wrist));
        buttonBoard.button(2).onTrue(new GoToL1(elevator, wrist));
        buttonBoard.button(3).onTrue(new GoToL2(elevator, wrist));
        buttonBoard.button(4).onTrue(new GoToL3(elevator, wrist));
        buttonBoard.button(5).onTrue(new GoToL4(elevator, wrist));
        buttonBoard.button(6).onTrue(new GoHomeL1Only(wrist, elevator));
        buttonBoard.button(7).onTrue(new RunAlgae2(elevator, wrist, intake));
        buttonBoard.button(8).onTrue(new RunAlgae1(elevator, wrist, intake));

        //driver controls
        joystick.y().onTrue(new ScoreL1(elevator, wrist, intake));
        joystick.a().onTrue(new RunIntakeOut(intake));
        joystick.x().onTrue(new ScoreL4(wrist, elevator));
        joystick.b().onTrue(new ScoreL2_3(elevator, wrist, intake));
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //left bumper to toggle drvetrain to low speed
        joystick.leftBumper().whileTrue(new InstantCommand(() -> translationMultiplier = .5));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> translationMultiplier = 1.0));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> strafeMultiplier = .5));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> strafeMultiplier = 1.0));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> rotateMultiplier = .5));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> rotateMultiplier = 1.0));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected(); 
            
        }
    }

