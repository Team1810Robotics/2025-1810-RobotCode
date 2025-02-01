// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
//import frc.robot.commands.Intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xbox = new CommandXboxController(0);

    private final CommandJoystick joystick = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureXbox();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    @SuppressWarnings("unused")
    private void configureXbox() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((deadzone(-xbox.getLeftY()) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((deadzone(-xbox.getLeftX()) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(deadzone(-visionSubsystem.visionTargetPIDCalc(xbox.getRightX(), xbox.a().getAsBoolean())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        xbox.b().whileTrue(drivetrain.applyRequest(() -> brake));
        xbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))
        ));

        //Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xbox.back().and(xbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        xbox.back().and(xbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        xbox.start().and(xbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        xbox.start().and(xbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        //xbox.rightTrigger(0.03).onTrue(new InstantCommand(() -> intakeSubsystem.setSpeed(xbox.getRawAxis(3))));
        //xbox.leftTrigger(0.03).onTrue(new InstantCommand(() -> intakeSubsystem.setSpeed(-xbox.getRawAxis(2))));

        //xbox.leftTrigger().whileTrue(new Intake(intakeSubsystem, true));
        //xbox.rightTrigger().whileTrue(new Intake(intakeSubsystem, false));
    }

    @SuppressWarnings("unused")
    private void configureJoystick() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((deadzone(-joystick.getY()) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((deadzone(-joystick.getX()) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(deadzone(visionSubsystem.visionTargetPIDCalc(joystick.getZ(), joystick.button(0).getAsBoolean())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.button(1).whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.button(11).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.button(10).and(joystick.button(3)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.button(10).and(joystick.button(3)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.button(9).and(joystick.button(4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.button(9).and(joystick.button(4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.button(12).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
   
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Applies a deadzone to the input value. Values within the range of -0.1 to 0.1 are
     * set to zero to prevent small unintentional movements. Values outside this range
     * are squared to maintain the direction while providing finer control at lower speeds.
     *
     * @param input the input value to be adjusted
     * @return the adjusted value after applying the deadzone and squaring
     */
    public double deadzone(double input) {
        //TODO: Work with drivers to find deadzone
        if (Math.abs(input) <= .1 && Math.abs(input) > 0) {
            return 0;
        }

        
        if (input < 0) {
            return input * -input;
        } else {
            return input * input;
        }
         
    }
}
