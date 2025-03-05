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

import com.pathplanner.lib.auto.NamedCommands;

@SuppressWarnings("unused") // For now :) 
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond)*1.5; // 3/4 of a rotation per second max angular velocity

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

    public double currentPitch;
    public double currentArm;
    public double currentExtenstion;

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private GenericEntry armSetpoint =
    tab.add("Arm Setpoint", 0)
       .getEntry();

    public RobotContainer() {
        // intakeSubsystem.setDefaultCommand(new Intake(intakeSubsystem, Mode.IDLE));
        configureBindings();
        addNamedCommands();
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
                drive.withVelocityX((visionSubsystem.visionDrive(driverXbox.getLeftY(), 0.3, visionSubsystem.getRange().get(), driverXbox.x().getAsBoolean(), visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
                    .withVelocityY((visionSubsystem.visionDrive(driverXbox.getLeftX(), 0.0, -visionSubsystem.getYaw().get(), driverXbox.b().getAsBoolean(), visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
                    .withRotationalRate(-visionSubsystem.visionTargetPIDCalc(-driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        );

        driverXbox.rightTrigger().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(driverXbox.getLeftY() * MaxSpeed / 8).withVelocityY(driverXbox.getLeftX() * MaxSpeed / 8).withRotationalRate(driverXbox.getRightX() * MaxSpeed / 8))
        );


/*         drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-visionSubsystem.visionDrive(driverXbox.getLeftY(), 0.3, visionSubsystem.getRange().get(), driverXbox.b().getAsBoolean(), visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
                    .withVelocityY((-visionSubsystem.visionDrive(driverXbox.getLeftX(), 0.0, -visionSubsystem.getYaw().get(), driverXbox.y().getAsBoolean(), visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
                    .withRotationalRate(-visionSubsystem.visionTargetPIDCalc(-driverXbox.getRightX(), driverXbox.a().getAsBoolean()) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            );  */

        //Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
         driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         driverXbox.start().and(driverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
         driverXbox.start().and(driverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        manipulatorXbox.a().onTrue(l1Position());
        manipulatorXbox.b().onTrue(l2Position());
        manipulatorXbox.x().onTrue(l3Position());
        manipulatorXbox.y().onTrue(l4Position());
        manipulatorXbox.back().onTrue(algaeL3Position());
        manipulatorXbox.rightBumper().onTrue(intakePostition()); //Base
        manipulatorXbox.back().onTrue(groundPickup());
        manipulatorXbox.leftBumper().onTrue(basePosition());
        // manipulatorXbox.start().onTrue(flipIntake());

        // driverXbox.a().whileTrue(new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT));
        // driverXbox.y().whileTrue(new Extender(extenderSubsystem, ExtenderConstants.L2_HEIGHT));

        // manipulatorXbox.a().onTrue(new Roll(rollSubsystem, RollConstants.L2_POSITION));
        // manipulatorXbox.y().onTrue(new Roll(rollSubsystem, RollConstants.INTAKE_POSITION));

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

    public double getSetpoint(){
        return armSetpoint.get().getDouble();
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
        return autoChooser.getSelected();
    }

    public void addNamedCommands(){
        NamedCommands.registerCommand("Base Position", basePosition());
        NamedCommands.registerCommand("L1 Position", l1Position());
        NamedCommands.registerCommand("L2 Position", l2Position());
        NamedCommands.registerCommand("L3 Position", l3Position());
        NamedCommands.registerCommand("L4 Position", l4Position());
        //NamedCommands.registerCommand("Ground Pickup", groundPickup());
        NamedCommands.registerCommand("Intake Position", intakePostition());
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

