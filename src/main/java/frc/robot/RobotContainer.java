package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.RobotStates;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.superstructure.ArmSubsystem;
import frc.robot.subsystems.superstructure.ExtenderSubsystem;
import frc.robot.subsystems.superstructure.IntakeSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;
import frc.robot.util.Poses;
import frc.robot.util.Telemetry;
import frc.robot.util.constants.TunerConstants;
import frc.robot.util.constants.RobotConstants.VisionConstants;
import frc.robot.util.constants.RobotConstants.IntakeConstants.IntakeMode;
import frc.robot.subsystems.superstructure.SuperstructureState;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond) * 1.5; 

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

    private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final VisionSubsystem visionLeft = new VisionSubsystem(VisionConstants.LEFT_CAMERA, VisionConstants.CAMERA_TO_ROBOT_LEFT);
    private final VisionSubsystem visionRight = new VisionSubsystem(VisionConstants.RIGHT_CAMERA, VisionConstants.CAMERA_TO_ROBOT_RIGHT);

    private static final ArmSubsystem armSubsystem = new ArmSubsystem();
    private static final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    private static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private static final PitchSubsystem pitchSubsystem = new PitchSubsystem();
    private static final RollSubsystem rollSubsystem = new RollSubsystem();

    private static final Superstructure superstructure = new Superstructure(pitchSubsystem, rollSubsystem, extenderSubsystem, armSubsystem, intakeSubsystem);


    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        addNamedCommands();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        Shuffleboard.getTab("Teleoperated").addString("Robot State", () -> RobotState.getRobotState().toString());
        Shuffleboard.getTab("Teleoperated").addString("Superstructure State", () -> Superstructure.getCurrentSuperstructureState().toString());
        Shuffleboard.getTab("Teleoperated").addBoolean("Should Eject", RobotState.shouldEject);
        Shuffleboard.getTab("Teleoperated").addString("State Override", () -> RobotState.stateIsOverride.getAsBoolean() ? "States overriden, original controls" : "State Based Controls");

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

        //Bindings for no pieces
        manipulatorXbox.a().and(RobotState.stateIsNone).onTrue(coralStation());
        manipulatorXbox.b().and(RobotState.stateIsNone).onTrue(groundPickup());
        manipulatorXbox.x().and(RobotState.stateIsNone).onTrue(algaePickup());
        manipulatorXbox.y().and(RobotState.stateIsNone).onTrue(algaeClear());

        //Bindings for coral
        manipulatorXbox.a().and(RobotState.stateIsCoral).onTrue(l1());
        manipulatorXbox.b().and(RobotState.stateIsCoral).onTrue(l2());
        manipulatorXbox.x().and(RobotState.stateIsCoral).onTrue(l3());
        manipulatorXbox.y().and(RobotState.stateIsCoral).onTrue(l4());

        //Bindings for algae
        manipulatorXbox.a().and(RobotState.stateIsAlgae).onTrue(processor());
        manipulatorXbox.b().and(RobotState.stateIsAlgae).onTrue(net());

        manipulatorXbox.a().and(RobotState.stateIsOverride).onTrue(l1());
        manipulatorXbox.b().and(RobotState.stateIsOverride).onTrue(l2());
        manipulatorXbox.x().and(RobotState.stateIsOverride).onTrue(l3());
        manipulatorXbox.y().and(RobotState.stateIsOverride).onTrue(l4());

        manipulatorXbox.leftBumper().and(RobotState.stateIsOverride).onTrue(base());
        manipulatorXbox.rightBumper().and(RobotState.stateIsOverride).onTrue(coralStation());
        manipulatorXbox.start().and(RobotState.stateIsOverride).onTrue(groundPickup());
        manipulatorXbox.leftStick().and(RobotState.stateIsOverride).onTrue(algaeClear());


        manipulatorXbox.povUp().whileTrue(extenderSubsystem.runManual(.5));
        manipulatorXbox.povDown().whileTrue(extenderSubsystem.runManual(-.5));

        manipulatorXbox.rightTrigger().onTrue(intake());
        manipulatorXbox.leftTrigger().whileTrue(outtake());

        manipulatorXbox.back().onTrue(Commands.runOnce(() -> RobotState.updateState(RobotStates.NONE)));

        manipulatorXbox.leftStick().and(() -> !RobotState.stateIsOverride.getAsBoolean()).onTrue(Commands.runOnce(() -> RobotState.updateState(RobotStates.OVERRIDE)));
    }

    public void addNamedCommands() {
        NamedCommands.registerCommand("Intake", coralStation().withTimeout(1));
        NamedCommands.registerCommand("Base", base().withTimeout(2.3));
        NamedCommands.registerCommand("L2", l2().withTimeout(2));
        NamedCommands.registerCommand("L3", l3().withTimeout(2));
        NamedCommands.registerCommand("L4", l4().withTimeout(3.5));

        NamedCommands.registerCommand("Outtake", outtake());
        NamedCommands.registerCommand("Intake", intake());

        NamedCommands.registerCommand("Left Align", leftAlign().withTimeout(2)); 
        NamedCommands.registerCommand("Right Align", rightAlign().withTimeout(2));
    }

    public Command configureAuto() {
        if (autoChooser.getSelected() != null) {
            return autoChooser.getSelected();
        } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");    
        }
    }
            
    private Command coralStation() {
        return superstructure.applyTargetStateParallel(SuperstructureState.CORAL_STATION);
    }

    private Command l1() {
        return superstructure.applyTargetStateParallel(SuperstructureState.L1);
    }

    private Command l2() {
        return superstructure.applyTargetStateParallel(SuperstructureState.L2);
    }

    private Command l3() {
        return superstructure.applyTargetStateParallel(SuperstructureState.L3);
    }

    private Command l4() {
        return superstructure.applyTargetStateParallel(SuperstructureState.L4);
    }

    private Command base() {
        return superstructure.applyTargetStateParallel(SuperstructureState.BASE);
    }

    private Command groundPickup() {
        return superstructure.applyTargetStateParallel(SuperstructureState.GROUND_PICKUP);
    }

    private Command processor() {
        return superstructure.applyTargetStateParallel(SuperstructureState.PROCESSOR);
    }

    private Command net() {
        return superstructure.applyTargetStateParallel(SuperstructureState.NET);
    }

    private Command algaePickup() {
        Optional<Integer> id = visionLeft.getTargetID();

        if (id.isEmpty()) id = visionRight.getTargetID();

        if (id.isEmpty()) {
            DriverStation.reportWarning("Attempted to pick up algae, no tag found", null);
            return new InstantCommand();
        } else if (VisionConstants.LOW_ALGAE_TAGS.contains(id.get())) {
            return superstructure.applyTargetStateParallel(SuperstructureState.LOW_ALGAE_PICKUP);
        } else if (VisionConstants.HIGH_ALGAE_TAGS.contains(id.get())) {
            return superstructure.applyTargetStateParallel(SuperstructureState.HIGH_ALGAE_PICKUP);
        } else {
            DriverStation.reportWarning("Attempted to pick up algae, tag not recognized", null);
            return new InstantCommand();
        }
    }

    private Command algaeClear() {
        Optional<Integer> id = visionLeft.getTargetID();

        if (id.isEmpty()) id = visionRight.getTargetID();
        

        if (id.isEmpty()) {
            DriverStation.reportWarning("Attempted to clear algae, no tag found", null);
            return new InstantCommand();
        } else if (VisionConstants.LOW_ALGAE_TAGS.contains(id.get())) {
            return superstructure.applyTargetStateParallel(SuperstructureState.LOW_ALGAE_CLEAR);
        } else if (VisionConstants.HIGH_ALGAE_TAGS.contains(id.get())) {
            return superstructure.applyTargetStateParallel(SuperstructureState.HIGH_ALGAE_CLEAR);
        } else {
            DriverStation.reportWarning("Attempted to clear algae, tag not recognized", null);
            return new InstantCommand();
        }
    }

    private Command intake() {
        return intakeSubsystem.run(IntakeMode.IN);
    }

    private Command outtake() {
        return intakeSubsystem.run(IntakeMode.OUT);
    }

    private Command leftAlign() {
        if (visionRight.getTargetID().isEmpty()) return new InstantCommand();
        
        return new DriveToPose(
            drivetrain, 
            visDrive, 
            Poses.getScorePose(
                visionRight.getTargetID().get(),
                true, 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            )
        );
    }

    private Command rightAlign() {
        if (visionLeft.getTargetID().isEmpty()) return new InstantCommand();
        
        return new DriveToPose(
            drivetrain, 
            visDrive, 
            Poses.getScorePose(
                visionLeft.getTargetID().get(), 
                false, 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            )
        );
    }

    public static ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public static ExtenderSubsystem getExtenderSubsystem() {
        return extenderSubsystem;
    }

    public static IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public static PitchSubsystem getPitchSubsystem() {
        return pitchSubsystem;
    }

    public static RollSubsystem getRollSubsystem() {
        return rollSubsystem;
    }

    public static CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}