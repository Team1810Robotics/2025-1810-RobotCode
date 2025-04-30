package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.States;
import frc.robot.commands.DriveToPose;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.constants.RobotConstants.IntakeConstants.IntakeMode;
import frc.robot.constants.RobotConstants.SuperstructueConstants.SuperstructureState;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.superstructure.ArmSubsystem;
import frc.robot.subsystems.superstructure.ExtenderSubsystem;
import frc.robot.subsystems.superstructure.IntakeSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;


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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final static IntakeSubsystem intakeSubsystem = Superstructure.getInstance().getIntakeSubsystem();
    public final VisionSubsystem visionLeft = new VisionSubsystem(VisionConstants.LEFT_CAMERA, VisionConstants.CAMERA_TO_ROBOT_LEFT, drivetrain);
    public final VisionSubsystem visionRight = new VisionSubsystem(VisionConstants.RIGHT_CAMERA, VisionConstants.CAMERA_TO_ROBOT_RIGHT, drivetrain);
    public final static ArmSubsystem armSubsystem = Superstructure.getInstance().getArmSubsystem();
    public final static PitchSubsystem pitchSubsystem = Superstructure.getInstance().getPitchSubsystem();
    public final static RollSubsystem rollSubsystem = Superstructure.getInstance().getRollSubsystem();
    public final static ExtenderSubsystem extenderSubsystem = Superstructure.getInstance().getExtenderSubsystem();


    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        addNamedCommands();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        Shuffleboard.getTab("Teleop").addNumber("Battery Voltage", () -> RobotController.getBatteryVoltage());

        Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble());

        Shuffleboard.getTab("Teleoperated").addString("Robot State", () -> RobotState.getRobotState().toString());
        Shuffleboard.getTab("Teleoperated").addString("Superstructure State", () -> Superstructure.getCurrentSuperstructureState().toString());
        Shuffleboard.getTab("Teleoperated").addBoolean("Should Eject", RobotState.shouldEject);

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


        manipulatorXbox.a().and(RobotState.stateIsNone).onTrue(intakePostition());
        manipulatorXbox.b().and(RobotState.stateIsNone).onTrue(groundPickup());
        manipulatorXbox.x().and(RobotState.stateIsNone).onTrue(algaePickup());
        manipulatorXbox.y().and(RobotState.stateIsNone).onTrue(algaeClear());

        manipulatorXbox.a().and(RobotState.stateIsCoral).onTrue(l1Position());
        manipulatorXbox.b().and(RobotState.stateIsCoral).onTrue(l2Position());
        manipulatorXbox.x().and(RobotState.stateIsCoral).onTrue(l3Position());
        manipulatorXbox.y().and(RobotState.stateIsCoral).onTrue(l4Position());

        manipulatorXbox.a().and(RobotState.stateIsAlgae).onTrue(processor());
        manipulatorXbox.b().and(RobotState.stateIsAlgae).onTrue(net());


        manipulatorXbox.povUp().whileTrue(extenderSubsystem.runManual(.5));
        manipulatorXbox.povDown().whileTrue(extenderSubsystem.runManual(-.5));

        manipulatorXbox.rightTrigger().onTrue(intake());
        manipulatorXbox.leftTrigger().whileTrue(outtake());

        manipulatorXbox.start().onTrue(Commands.runOnce(() -> RobotState.updateState(States.NONE)));
    }

    public void addNamedCommands() {
        NamedCommands.registerCommand("Intake", intakePostition().withTimeout(1));
        NamedCommands.registerCommand("Base", basePosition().withTimeout(2.3));
        NamedCommands.registerCommand("L2", l2Position().withTimeout(2));
        NamedCommands.registerCommand("L3", l3Position().withTimeout(2));
        NamedCommands.registerCommand("L4", l4Position().withTimeout(3.5));

        NamedCommands.registerCommand("Outtake", outtake());
        NamedCommands.registerCommand("Intake", intake());

        NamedCommands.registerCommand("Left Align", leftAlign().withTimeout(2)); 
        NamedCommands.registerCommand("Right Align", rightAlign().withTimeout(2));

        NamedCommands.registerCommand("End", new RunCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }

    public Command configureAuto() {
        if (autoChooser.getSelected() != null) {
            return autoChooser.getSelected();
        } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");
        }
    }
    public Command intakePostition() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.CORAL_STATION);
    }

    public Command l1Position() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.L1);
    }

    public Command l2Position() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.L2);
    }

    public Command l3Position() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.L3);
    }

    public Command l4Position() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.L4);
    }

    public Command basePosition() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.BASE);
    }

    public Command groundPickup() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.GROUND_PICKUP);
    }

    public Command processor() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.PROCESSOR);
    }

    public Command net() {
        return Superstructure.getInstance().applyTargetState(SuperstructureState.NET);
    }


    public Command algaePickup() {
        int id = visionLeft.getTargetID().orElse(visionRight.getTargetID().orElse(-1));

        if (id == -1) {
            DriverStation.reportWarning("Attempted to pick up algae, no tag found", null);
            return new InstantCommand();
        } else if (VisionConstants.LOW_ALGAE_TAGS.contains(id)) {
            return Superstructure.getInstance().applyTargetState(SuperstructureState.LOW_ALGAE_PICKUP);
        } else if (VisionConstants.HIGH_ALGAE_TAGS.contains(id)) {
            return Superstructure.getInstance().applyTargetState(SuperstructureState.HIGH_ALGAE_PICKUP);
        } else {
            DriverStation.reportWarning("Attempted to pick up algae, tag not recognized", null);
            return new InstantCommand();
        }
    }

    public Command algaeClear() {
        int id = visionLeft.getTargetID().orElse(visionRight.getTargetID().orElse(-1));

        if (id == -1) {
            DriverStation.reportWarning("Attempted to clear algae, no tag found", null);
            return new InstantCommand();
        } else if (VisionConstants.LOW_ALGAE_TAGS.contains(id)) {
            return Superstructure.getInstance().applyTargetState(SuperstructureState.LOW_ALGAE_CLEAR);
        } else if (VisionConstants.HIGH_ALGAE_TAGS.contains(id)) {
            return Superstructure.getInstance().applyTargetState(SuperstructureState.HIGH_ALGAE_CLEAR);
        } else {
            DriverStation.reportWarning("Attempted to clear algae, tag not recognized", null);
            return new InstantCommand();
        }
    }

    public Command intake() {
        return intakeSubsystem.run(IntakeMode.IN);
    }

    public Command outtake() {
        return intakeSubsystem.run(IntakeMode.OUT);
    }

    public Command leftAlign() {
        if (visionLeft.getTargetID().isEmpty()) return new InstantCommand();
        
        return new DriveToPose(
            drivetrain, 
            visDrive, 
            Reef.getScorePose(
                    visionRight.getTargetID().get(), 
                    true, 
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            )
        );
    }

    public Command rightAlign() {
        if (visionRight.getTargetID().isEmpty()) return new InstantCommand();
        
        return new DriveToPose(
            drivetrain, 
            visDrive, 
            Reef.getScorePose(
                    visionLeft.getTargetID().get(), 
                    true, 
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            )
        );
    }

}