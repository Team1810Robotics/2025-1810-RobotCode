package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.security.cert.CertPathValidatorException.BasicReason;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.ExtenderConstants.ExtenderHeights;
import frc.robot.commands.Arm;
import frc.robot.commands.Extender;
import frc.robot.commands.Intake;
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

@SuppressWarnings("unused") // For now :) 
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController manipulatorXbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static ArmSubsystem armSubsystem = new ArmSubsystem();
    public final static ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    public final static LedSubsystem ledSubsystem = new LedSubsystem();
    public final static PitchSubsystem pitchSubsystem = new PitchSubsystem();
    public final static RollSubsystem rollSubsystem = new RollSubsystem();

    private final SendableChooser<Command> autoChooser;

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private GenericEntry armSetpoint =
    tab.add("Arm Setpoint", 0)
       .getEntry();

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        Shuffleboard.getTab("Arm").add(armSubsystem);
        Shuffleboard.getTab("Arm").addNumber("Arm Setpoint Actual", ()-> armSetpoint.get().getDouble());

    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-visionSubsystem.visionDrive(driverXbox.getLeftY(), 0.3, visionSubsystem.getRange().get(), driverXbox.b().getAsBoolean(), visionSubsystem.driveControllerY) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((-visionSubsystem.visionDrive(driverXbox.getLeftX(), 0.0, -visionSubsystem.getYaw().get(), driverXbox.y().getAsBoolean(), visionSubsystem.driveControllerX) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(-visionSubsystem.visionTargetPIDCalc(driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            );

/*      xbox.rightBumper().whileTrue(new ArmCommand(armSubsystem, getSetpoint()));
        xbox.leftBumper().whileTrue(new ArmCommand(armSubsystem, 0));
        xbox.leftStick().whileTrue(new ArmCommand(armSubsystem, 10)); */

        // xbox.rightBumper().whileTrue(new Pitch(pitchSubsystem, 135));
        // xbox.leftBumper().whileTrue(new Pitch(pitchSubsystem, 80));
        // xbox.rightTrigger().whileTrue(new Pitch(pitchSubsystem, 0));
        
        driverXbox.x().whileTrue(new Extender(extenderSubsystem, ExtenderHeights.L2));
        driverXbox.b().whileTrue(new Extender(extenderSubsystem, ExtenderHeights.L4));

        driverXbox.y().whileTrue(new Arm(armSubsystem, 90));
        
        //Reset Gyro

        driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(logger::telemeterize);
        
        // xbox.rightTrigger(0.03).onTrue(new InstantCommand(() -> intakeSubsystem.setSpeed(xbox.getRawAxis(3))));
        // xbox.leftTrigger(0.03).onTrue(new InstantCommand(() -> intakeSubsystem.setSpeed(-xbox.getRawAxis(2))));
        
        // xbox.leftTrigger().whileTrue(new Intake(intakeSubsystem, GamePiece.CORAL));
        // xbox.rightTrigger().whileTrue(new Intake(intakeSubsystem, GamePiece.ALGAE));
        // xbox.a().whileTrue(new Intake(intakeSubsystem, GamePiece.NONE));
    }

    public double getSetpoint(){
        return armSetpoint.get().getDouble();
    }
        
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
