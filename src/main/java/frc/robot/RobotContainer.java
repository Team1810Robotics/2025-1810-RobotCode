package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Field;
import java.security.cert.CertPathValidatorException.BasicReason;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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
        // intakeSubsystem.setDefaultCommand(new Intake(intakeSubsystem, Mode.IDLE));
        configureBindings();
        configureVisionPoseEstimation();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        Shuffleboard.getTab("Arm").add(armSubsystem);
        Shuffleboard.getTab("Arm").addNumber("Arm Setpoint Actual", ()-> armSetpoint.get().getDouble());

        Shuffleboard.getTab("Swerve").addNumber("S Velo", () -> drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());

        Shuffleboard.getTab("Swerve").addNumber("FL Turn Error", () -> drivetrain.getState().ModulePositions[2].angle.getDegrees());
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-visionSubsystem.visionDrive(driverXbox.getLeftY(), 0.3, visionSubsystem.getRange().get(), driverXbox.b().getAsBoolean(), visionSubsystem.driveControllerY) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((-visionSubsystem.visionDrive(driverXbox.getLeftX(), 0.0, -visionSubsystem.getYaw().get(), driverXbox.y().getAsBoolean(), visionSubsystem.driveControllerX) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(-visionSubsystem.visionTargetPIDCalc(-driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            );

        //Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverXbox.start().and(driverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverXbox.start().and(driverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

/*      xbox.rightBumper().whileTrue(new ArmCommand(armSubsystem, getSetpoint()));
        xbox.leftBumper().whileTrue(new ArmCommand(armSubsystem, 0));
        xbox.leftStick().whileTrue(new ArmCommand(armSubsystem, 10)); */

        // driverXbox.x().onTrue(new Roll(rollSubsystem, 250));
        // driverXbox.b().onTrue(new Roll(rollSubsystem, 160));

        // driverXbox.rightTrigger().onTrue(basePosiiton());
        // driverXbox.rightBumper().onTrue(intakePostition());
        // driverXbox.leftBumper().onTrue(l3Position());


        manipulatorXbox.leftBumper().whileTrue(new Arm(armSubsystem, ArmConstants.BASE_POSITION));

        // manipulatorXbox.b().or(manipulatorXbox.a()).whileTrue(drivetrain.applyRequest(() -> brake));
        // manipulatorXbox.a().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(0, 1))
        // ));
        // manipulatorXbox.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(1, 0))
        // ));
        
        manipulatorXbox.a().whileTrue(new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT));
        manipulatorXbox.x().whileTrue(new Extender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));
        manipulatorXbox.b().whileTrue(new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT));
        manipulatorXbox.y().whileTrue(new Extender(extenderSubsystem, ExtenderConstants.L4_HEIGHT));

        // driverXbox.button(10).whileTrue(new Arm(armSubsystem, 120));
        // driverXbox.button(9).whileTrue(new Arm(armSubsystem, 45));
        // driverXbox.leftTrigger().whileTrue(new Arm(armSubsystem, 115));
        
        //Reset Gyro
        driverXbox.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(logger::telemeterize);
        
        // xbox.rightTrigger(0.03).onTrue(new InstantCommand(() -> intakeSubsystem.setSpeed(xbox.getRawAxis(3))));
        // xbox.leftTrigger(0.03).onTrue(new InstantCommand(() -> intakeSubsystem.setSpeed(-xbox.getRawAxis(2))));
        
        driverXbox.x().whileTrue(new Intake(intakeSubsystem, Mode.CORAL));
        // driverXbox.x().toggleOnTrue(new Intake(intakeSubsystem, Mode.ALGAE));
        driverXbox.b().whileTrue(new Intake(intakeSubsystem, Mode.OUT));
    }

    public double getSetpoint(){
        return armSetpoint.get().getDouble();
    }

    public Command intakePostition() {
        return new Arm(armSubsystem, ArmConstants.INTAKE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION), (new Pitch(pitchSubsystem, PitchConstants.INTAKE_POSITION)));
    }

    public Command l1Position() {
        return new Arm(armSubsystem, ArmConstants.L1_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION).alongWith(new Pitch(pitchSubsystem, PitchConstants.L1_POSITION)));
    }

    public Command l2Position() {
        return new Arm(armSubsystem, ArmConstants.L2_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.SCORE_POSITION), new Pitch(pitchSubsystem, PitchConstants.L2_POSITION));
    }

    public Command l3Position() {
        return new Arm(armSubsystem, ArmConstants.L3_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.SCORE_POSITION), new Pitch(pitchSubsystem, PitchConstants.L3_POSITION));
    }

    public Command l4Position() {
        return new Arm(armSubsystem, ArmConstants.L4_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.SCORE_POSITION), new Pitch(pitchSubsystem, PitchConstants.L4_POSITION));
    }

    public Command basePosiiton() {
        return new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION), (new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION)));
    }
        
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void addNamedCommands(){
        
    }

    private void configureVisionPoseEstimation() {
        // Run vision-based pose estimation on a separate thread
        new Thread(() -> {
            while (true) {
                // Get the current estimated pose from the drivetrain
                Pose2d currentPose = drivetrain.getState().Pose;
                
                // Get vision-estimated pose
                Optional<EstimatedRobotPose> visionEstimate = 
                    visionSubsystem.getEstimatedGlobalPose(currentPose);
                
                // If we have a vision measurement, add it to the drivetrain
                if (visionEstimate.isPresent()) {
                    EstimatedRobotPose pose = visionEstimate.get();
                    
                    // Timestamped pose from PhotonVision
                    Pose2d visionPose = pose.estimatedPose.toPose2d();
                    double timestamp = pose.timestampSeconds;
                    
                    // Standard deviations based on target distance
                    // The further the target, the higher the standard deviation
                    double distance = visionSubsystem.getRange().orElse(5.0);
                    double xStdDev = 0.1 + distance * 0.05; // Higher std dev with distance
                    double yStdDev = 0.1 + distance * 0.05;
                    double thetaStdDev = 0.1 + distance * 0.01;
                    
                    // Create standard deviation matrix for vision measurement
                    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xStdDev, yStdDev, thetaStdDev);
                    
                    // Add vision measurement to drivetrain pose estimator
                    drivetrain.addVisionMeasurement(visionPose, timestamp, visionStdDevs);
                }
                
                // Sleep to avoid overwhelming CPU
                try {
                    Thread.sleep(20); // 50Hz update rate
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }
}
