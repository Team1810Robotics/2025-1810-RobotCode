package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Field;
import java.security.cert.CertPathValidatorException.BasicReason;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

//import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

@SuppressWarnings("unused") // For now :) 
public class RobotContainer {

    private int ethanCulver;
    private Field2d field2d;

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
    private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle();

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

    private final SendableChooser<Command> autoChooser ;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public RobotContainer() {
        ethanCulver = 0;

        field2d = new Field2d();
        field2d.setRobotPose(getPose());
        SmartDashboard.putData("Field", field2d);

        addNamedCommands();
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("");
        configureAutonomus();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        Shuffleboard.getTab("Arm").add("Arm Subsystem", armSubsystem);

        Shuffleboard.getTab("Teleop").addNumber("Battery Voltage",() -> RobotController.getBatteryVoltage());
        Shuffleboard.getTab("Teleoperated").addNumber("ETHAN CULVER WILL FEEL MY VENGANCE", () -> ethanCulver);

        Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble());

        Shuffleboard.getTab("Vision").addNumber("VisPara", () -> visionSubsystem.visionPara(0, true, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 8));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-driverXbox.getLeftY() * MaxSpeed) / 4)
                    .withVelocityY((-driverXbox.getLeftX() * MaxSpeed) / 4)
                    .withRotationalRate(driverXbox.getRightX() * MaxAngularRate))
        );

        driverXbox.rightTrigger().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed / 1).withVelocityY(-driverXbox.getLeftX() * MaxSpeed / 1).withRotationalRate(driverXbox.getRightX() * MaxSpeed / 1))
        );

        // driverXbox.a().whileTrue(
        //     drivetrain.applyRequest(() -> visDrive.withRotationalRate(-visionSubsystem.visionPara(driverXbox.getRightX(), true, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 8)))
        // );

        driverXbox.leftTrigger().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed / 8).withVelocityY(-driverXbox.getLeftX() * MaxSpeed / 8).withRotationalRate(driverXbox.getRightX() * MaxAngularRate / 8))
        );

        driverXbox.b().whileTrue(
            drivetrain.applyRequest(() -> visDrive.withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.05, true, visionSubsystem.driveControllerYRight) * MaxSpeed) / 4) // Drive forward with negative Y (forward)
                .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerXRight) * MaxSpeed) / 4) // Drive left with negative X (left)
                .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        );

        driverXbox.x().whileTrue(
            drivetrain.applyRequest(() -> visDrive.withVelocityX((visionSubsystem.visionXDriveRight(driverXbox.getLeftY(), -0.5, true, visionSubsystem.driveControllerYLeft) * MaxSpeed) / 4) // Drive forward with negative Y (forward)
                .withVelocityY((-visionSubsystem.visionYDriveRight(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerXLeft) * MaxSpeed) / 4) // Drive left with negative X (left)
                .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        );

        driverXbox.x().or(driverXbox.b()).onTrue(Commands.runOnce(() -> ethanCulver++));
        

        manipulatorXbox.a().onTrue(l1Position());
        manipulatorXbox.b().onTrue(l2Position());
        manipulatorXbox.x().onTrue(l3Position());
        manipulatorXbox.y().onTrue(l4Position());
        manipulatorXbox.back().onTrue(algaeL3Position());
        manipulatorXbox.rightBumper().onTrue(intakePostition()); //Base
        manipulatorXbox.back().onTrue(groundPickup());
        manipulatorXbox.leftBumper().onTrue(basePosition());


        driverXbox.a().and(driverXbox.start()).whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT));
        driverXbox.y().and(driverXbox.start()).whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.L4_HEIGHT));
        driverXbox.rightStick().onTrue(drivetrain.runOnce(() -> drivetrain.getPigeon2().setYaw(0)));
        manipulatorXbox.povUp().whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));
        manipulatorXbox.povDown().whileTrue(new ManualExtender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));

        manipulatorXbox.rightTrigger().onTrue(new Intake(intakeSubsystem, Mode.IN));
        manipulatorXbox.leftTrigger().whileTrue(new Intake(intakeSubsystem, Mode.OUT));
        
        //Reset Gyro
        driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public void addNamedCommands(){
        NamedCommands.registerCommand("Intake", intakePostition());
        NamedCommands.registerCommand("Base", basePosition());
        NamedCommands.registerCommand("L2", l2Position().withTimeout(2));
        NamedCommands.registerCommand("L3", l3Position().withTimeout(3.5));
        NamedCommands.registerCommand("L4", l4Position().withTimeout(5));

        NamedCommands.registerCommand("Score", new Intake(intakeSubsystem, Mode.OUT).withTimeout(1));
        NamedCommands.registerCommand("Intake", new Intake(intakeSubsystem, Mode.IN));

        //NamedCommands.registerCommand("Test",  drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(1).withRotationalRate(0)));
        NamedCommands.registerCommand("LeftTag", drivetrain.applyRequest(() -> visDrive.withVelocityX((visionSubsystem.visionXDriveRight(driverXbox.getLeftY(), -0.1, true, visionSubsystem.driveControllerYRight) * MaxSpeed) / 4) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDriveRight(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerXRight) * MaxSpeed) / 4)
            .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)).withTimeout(2));
        NamedCommands.registerCommand("RightTag",  drivetrain.applyRequest(() -> visDrive.withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.1, true, visionSubsystem.driveControllerYRight) * MaxSpeed) / 4) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerXRight) * MaxSpeed) / 4)
            .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)).withTimeout(2));
   
        NamedCommands.registerCommand("End", new RunCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }

    public Command configureAutonomus() {
        if (autoChooser.getSelected() != null){
            return autoChooser.getSelected();
        } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");
        }
    }  

    public Command intakePostition() {
        return new Arm(armSubsystem, ArmConstants.INTAKE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION), (new Pitch(pitchSubsystem, PitchConstants.INTAKE_POSITION)), (new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)));
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

    public Command outtake() {
        return new Intake(intakeSubsystem, Mode.OUT).alongWith(new Pitch(pitchSubsystem, pitchSubsystem.currentSetpoint), new Roll(rollSubsystem, rollSubsystem.currentSetpoint), new Extender(extenderSubsystem, extenderSubsystem.currentSetpoint));
    }

    private Pose2d getPose() {
        return drivetrain.getState().Pose;
    }
}