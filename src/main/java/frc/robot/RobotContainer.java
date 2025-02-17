// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xbox = new CommandXboxController(0);

    private final CommandJoystick joystick = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final SendableChooser<Command> autoChooser;

    double forward, strafe, turn, targetYaw, targetRange;

    public RobotContainer() {
        configureXbox();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    @SuppressWarnings("unused")
    private void configureXbox() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((deadzone(-visionSubsystem.visionDrive(xbox.getLeftY(), 0.5, visionSubsystem.getRange().get(), xbox.b().getAsBoolean(), visionSubsystem.driveControllerY)) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((deadzone(-xbox.getLeftX()) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(deadzone(-visionSubsystem.visionTargetPIDCalc(xbox.getRightX(), xbox.a().getAsBoolean())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

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

        xbox.leftTrigger().whileTrue(new Intake(intakeSubsystem, true));
        xbox.rightTrigger().whileTrue(new Intake(intakeSubsystem, false));
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
