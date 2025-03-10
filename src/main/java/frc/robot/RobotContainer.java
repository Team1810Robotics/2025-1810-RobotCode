package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Field;
import java.security.cert.CertPathValidatorException.BasicReason;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.IntakeConstants.Mode;
import frc.robot.Constants.WristConstants.PitchConstants;
import frc.robot.Constants.WristConstants.RollConstants;
import frc.robot.commands.Arm;
import frc.robot.commands.Extender;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualExtender;
import frc.robot.commands.Pitch;
import frc.robot.commands.Roll;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.RollSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AutoRoutines;

//import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

@SuppressWarnings("unused") // For now :) 
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond)*1.5; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric visDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandXboxController driverXbox = new CommandXboxController(0);
    public final static CommandXboxController manipulatorXbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static ArmSubsystem armSubsystem = new ArmSubsystem();
    public final static ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    //public final static LedSubsystem ledSubsystem = new LedSubsystem();
    public final static PitchSubsystem pitchSubsystem = new PitchSubsystem();
    public final static RollSubsystem rollSubsystem = new RollSubsystem();

    private final SendableChooser<Command> autoChooserS = new SendableChooser<>();

    public double currentPitch;
    public double currentArm;
    public double currentExtenstion;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain);

        autoFactory
        .bind("IN", new Intake(intakeSubsystem, Mode.IN))
        .bind("score", new Intake(intakeSubsystem, Mode.OUT))
        .bind("L2", l2Position());

        // intakeSubsystem.setDefaultCommand(new Intake(intakeSubsystem, Mode.IDLE));
        configureBindings();

        autoChooser.addRoutine("Line Test", autoRoutines::lineTest);
        autoChooser.addRoutine("Line Test Str", autoRoutines::lineTestStraight);
        autoChooser.addRoutine("Leave", autoRoutines::leave);
        autoChooser.addRoutine("Move", autoRoutines::move);
        autoChooser.addRoutine("Dis", autoRoutines::dis);
        //autoChooser.addRoutine("VisTest", autoRoutines::visionTest);
        autoChooser.addRoutine("Proper", autoRoutines::proper);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Shuffleboard.getTab("Arm").add("Arm Subsystem", armSubsystem);

        Shuffleboard.getTab("Swerve").addNumber("S Velo", () -> drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());

        Shuffleboard.getTab("Swerve").addNumber("FL Turn Error", () -> drivetrain.getState().ModulePositions[2].angle.getDegrees());

        Shuffleboard.getTab("Teleop").addNumber("Battery Voltage",() -> RobotController.getBatteryVoltage());

        //Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> drivetrain.getPigeon2().getAngle());
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-driverXbox.getLeftY() * MaxSpeed) / 4) // Drive forward with negative Y (forward)
                    .withVelocityY((-driverXbox.getLeftX() * MaxSpeed) / 4) // Drive left with negative X (left)
                    .withRotationalRate(driverXbox.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left
        );

        driverXbox.rightTrigger().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed / 1).withVelocityY(-driverXbox.getLeftX() * MaxSpeed / 1).withRotationalRate(driverXbox.getRightX() * MaxSpeed / 1))
        );

        driverXbox.leftTrigger().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed / 8).withVelocityY(-driverXbox.getLeftX() * MaxSpeed / 8).withRotationalRate(driverXbox.getRightX() * MaxAngularRate / 8))
        );

        driverXbox.x().whileTrue(
            drivetrain.applyRequest(() -> visDrive.withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.5, true, visionSubsystem.driveControllerY) * MaxSpeed) / 4) // Drive forward with negative Y (forward)
                .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerX) * MaxSpeed) / 4) // Drive left with negative X (left)
                .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        );

        driverXbox.b().whileTrue(
            drivetrain.applyRequest(() -> visDrive.withVelocityX((visionSubsystem.visionXDriveRight(driverXbox.getLeftY(), -0.5, true, visionSubsystem.driveControllerY) * MaxSpeed) / 4) // Drive forward with negative Y (forward)
                .withVelocityY((-visionSubsystem.visionYDriveRight(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerX) * MaxSpeed) / 4) // Drive left with negative X (left)
                .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        );

        manipulatorXbox.a().onTrue(l1Position());
        manipulatorXbox.b().onTrue(l2Position());
        // driverXbox.leftStick().onTrue(basePosition());
        manipulatorXbox.x().onTrue(l3Position());
        manipulatorXbox.y().onTrue(l4Position());
        manipulatorXbox.back().onTrue(algaeL3Position());
        manipulatorXbox.rightBumper().onTrue(intakePostition()); //Base
        manipulatorXbox.back().onTrue(groundPickup());
        manipulatorXbox.leftBumper().onTrue(basePosition());
        //manipulatorXbox.start().onTrue(flipIntake());

        driverXbox.a().and(driverXbox.start()).whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT));
        driverXbox.y().and(driverXbox.start()).whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));
        manipulatorXbox.povUp().whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));
        manipulatorXbox.povDown().whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));


        // manipulatorXbox.a().onTrue(new Roll(rollSubsystem, RollConstants.L2_POSITION));
        //manipulatorXbox.y().onTrue(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION));

        // manipulatorXbox.leftBumper().onTrue(new Pitch(pitchSubsystem, PitchConstants.UPRIGHT));

        // manipulatorXbox.b().onTrue(new Extender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));
        // manipulatorXbox.x().onTrue(new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT));

        manipulatorXbox.rightTrigger().onTrue(new Intake(intakeSubsystem, Mode.IN));
        manipulatorXbox.leftTrigger().whileTrue(new Intake(intakeSubsystem, Mode.OUT));
        
        //Reset Gyro
        driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }



    // public Command flipIntake(){
    //     return new Roll(rollSubsystem, RollConstants.UPSIDE_DOWN).alongWith(new Pitch(pitchSubsystem, currentPitch), new Arm(armSubsystem, currentArm), new Extender(extenderSubsystem, currentExtenstion));
    // }

    public Command visionDriveLeft() {
        return drivetrain.applyRequest(() ->
        drive.withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.05, true, visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
            .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
    }

    public Command visionDriveRight() {
        return drivetrain.applyRequest(() ->
        drive.withVelocityX((visionSubsystem.visionXDriveRight(driverXbox.getLeftY(), -0.05, true, visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDriveRight(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
            .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
    }

    public Command intakePostition() {
        return new Arm(armSubsystem, ArmConstants.INTAKE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION), (new Pitch(pitchSubsystem, PitchConstants.INTAKE_POSITION)), (new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)), updateCurrentPositions(currentArm, currentExtenstion, currentPitch));
    }

    public Command l1Position() {
        return new Arm(armSubsystem, ArmConstants.L1_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L1_POSITION), new Pitch(pitchSubsystem, PitchConstants.L1_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L1_HEIGHT));
    }

    public Command l2Position() {
        return new Arm(armSubsystem, ArmConstants.L2_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L2_POSITION), new Pitch(pitchSubsystem, PitchConstants.L2_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));
    }

    public Command l3Position() {
        return new Arm(armSubsystem, ArmConstants.L3_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L3_POSITION), new Pitch(pitchSubsystem, PitchConstants.L3_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT));
    }

    public Command l4Position() {
        return new Arm(armSubsystem, ArmConstants.L4_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L4_POSITION), new Pitch(pitchSubsystem, PitchConstants.L4_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L4_HEIGHT));
    }

    public Command basePosition() {
        return new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT));
    }

    public Command groundPickup() {
        return new Arm(armSubsystem, ArmConstants.GROUND_PICKUP).alongWith(new Roll(rollSubsystem, RollConstants.GROUND_PICKUP), new Pitch(pitchSubsystem, PitchConstants.GROUND_PICKUP), new Extender(extenderSubsystem, ExtenderConstants.GROUND_PICKUP));
    }

    public Command algaeL3Position() {
        return new Arm(armSubsystem, ArmConstants.L3_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION), new Pitch(pitchSubsystem, PitchConstants.L3_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT)); 
    }

    public Command updateCurrentPositions(double arm, double extend, double pitch) {
        return new RunCommand(() -> currentArm = arm).alongWith(new RunCommand(() -> currentExtenstion = extend), new RunCommand(() -> currentPitch = pitch));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }  
}

