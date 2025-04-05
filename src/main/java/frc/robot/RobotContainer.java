package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.superstructure.ArmSubsystem;
import frc.robot.subsystems.superstructure.ExtenderSubsystem;
import frc.robot.subsystems.superstructure.IntakeSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureMode;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond) * 1.5; // 3/4 of a rotation per second

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric visDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed); 

    public final static CommandXboxController driverXbox = new CommandXboxController(0);
    public final static CommandXboxController manipulatorXbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static ArmSubsystem armSubsystem = new ArmSubsystem();
    public final static ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    // public final static LedSubsystem ledSubsystem = new LedSubsystem();
    public final static PitchSubsystem pitchSubsystem = new PitchSubsystem();
    public final static RollSubsystem rollSubsystem = new RollSubsystem();

    public static final Superstructure superstructure = new Superstructure(extenderSubsystem, armSubsystem, pitchSubsystem, rollSubsystem, intakeSubsystem);

    private final SendableChooser<Command> autoChooser;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public RobotContainer() {
        addNamedCommands();
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("");
        configureAutonomus();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        Shuffleboard.getTab("Arm").add("Arm Subsystem", armSubsystem);

        Shuffleboard.getTab("Teleop").addNumber("Battery Voltage", () -> RobotController.getBatteryVoltage());

        Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX((-driverXbox.getLeftY() * MaxSpeed) / 4)
                        .withVelocityY((-driverXbox.getLeftX() * MaxSpeed) / 4)
                        .withRotationalRate(driverXbox.getRightX() * MaxAngularRate)));

        driverXbox.rightTrigger().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed / 2)
                        .withVelocityY(-driverXbox.getLeftX() * MaxSpeed / 2)
                        .withRotationalRate(driverXbox.getRightX() * MaxSpeed / 1.5)));


        driverXbox.leftTrigger().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed / 10)
                        .withVelocityY(-driverXbox.getLeftX() * MaxSpeed / 10)
                        .withRotationalRate(driverXbox.getRightX() * MaxAngularRate / 8)));

        driverXbox.b().whileTrue(rightAlign());
        driverXbox.x().whileTrue(leftAlign());

        driverXbox.rightStick().onTrue(drivetrain.runOnce(() -> drivetrain.getPigeon2().setYaw(0)));

        //Reset heading
        driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));



        manipulatorXbox.a().onTrue(l1Position());
        manipulatorXbox.b().onTrue(l2Position());
        manipulatorXbox.x().onTrue(l3Position());
        manipulatorXbox.y().onTrue(l4Position());
        manipulatorXbox.rightBumper().onTrue(intakePostition()); 
        manipulatorXbox.start().onTrue(groundPickup());
        manipulatorXbox.leftBumper().onTrue(basePosition());


        manipulatorXbox.povUp().whileTrue(extenderSubsystem.runManual(.5));
        manipulatorXbox.povDown().whileTrue(extenderSubsystem.runManual(-.5));

        manipulatorXbox.rightTrigger().onTrue(Commands.run(() -> intakeSubsystem.run(IntakeMode.IN), intakeSubsystem).until(() -> intakeSubsystem.end(IntakeMode.IN)));
        manipulatorXbox.leftTrigger().whileTrue(Commands.run(() -> intakeSubsystem.run(IntakeMode.OUT), intakeSubsystem).until(() -> intakeSubsystem.end(IntakeMode.OUT)));

        manipulatorXbox.button(10).onTrue(algae1());
        manipulatorXbox.button(9).onTrue(algae2());

    }

    public void addNamedCommands() {
        NamedCommands.registerCommand("Intake", intakePostition().withTimeout(1));
        NamedCommands.registerCommand("Base", basePosition().withTimeout(2.3));
        NamedCommands.registerCommand("L2", l2Position().withTimeout(2));
        NamedCommands.registerCommand("L3", l3Position().withTimeout(2));
        NamedCommands.registerCommand("L4", l4Position().withTimeout(3.5));


        NamedCommands.registerCommand("Outtake", Commands.run(() -> intakeSubsystem.run(IntakeMode.OUT), intakeSubsystem).until(() -> intakeSubsystem.end(IntakeMode.OUT)));
        NamedCommands.registerCommand("Intake", Commands.run(() -> intakeSubsystem.run(IntakeMode.IN), intakeSubsystem).until(() -> intakeSubsystem.end(IntakeMode.IN)));

        NamedCommands.registerCommand("Left Align", leftAlign().withTimeout(2)); 
        NamedCommands.registerCommand("Right Align", rightAlign().withTimeout(2));


        NamedCommands.registerCommand("End", new RunCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }

    public Command configureAutonomus() {
        if (autoChooser.getSelected() != null) {
            return autoChooser.getSelected();
        } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");
        }
    }
    public Command intakePostition() {
        return superstructure.applyRequest(SuperstructureMode.CORAL_STATION);
    }

    public Command l1Position() {
        return superstructure.applyRequest(SuperstructureMode.L1);
    }

    public Command l2Position() {
        return superstructure.applyRequest(SuperstructureMode.L2);
    }

    public Command l3Position() {
        return superstructure.applyRequest(SuperstructureMode.L3);
    }

    public Command l4Position() {
        return superstructure.applyRequest(SuperstructureMode.L4);
    }

    public Command basePosition() {
        return superstructure.applyRequest(SuperstructureMode.BASE);
    }

    public Command groundPickup() {
        return superstructure.applyRequest(SuperstructureMode.GROUND_PICKUP);
    }

    public Command algae1() {
        return superstructure.applyRequest(SuperstructureMode.ALGAE1);
    }

    public Command algae2() {
        return superstructure.applyRequest(SuperstructureMode.ALGAE2);
    }

    public Command leftAlign() {
        return drivetrain.applyRequest(() -> visDrive
                .withVelocityX((visionSubsystem.visionXDriveRight(driverXbox.getLeftY(), -0.1, true,
                        visionSubsystem.driveControllerYLeft) * MaxSpeed) / 4) 
                .withVelocityY((-visionSubsystem.visionYDriveRight(-driverXbox.getLeftX(), 0.0, true,
                        visionSubsystem.driveControllerXLeft) * MaxSpeed) / 4) 
                .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(),
                        driverXbox.a().getAsBoolean()) * MaxAngularRate));
    }

    public Command rightAlign() {
        return drivetrain.applyRequest(() -> visDrive
                .withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.1, true,
                        visionSubsystem.driveControllerYRight) * MaxSpeed) / 4) 
                .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true,
                        visionSubsystem.driveControllerXRight) * MaxSpeed) / 4) 
                .withRotationalRate(visionSubsystem.visionTargetPIDCalcLeft(driverXbox.getRightX(),
                        driverXbox.a().getAsBoolean()) * MaxAngularRate));
    }
}