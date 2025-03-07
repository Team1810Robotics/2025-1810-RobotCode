package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.RollSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoRoutines {
    private final AutoFactory m_factory;
    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    RollSubsystem rollSubsystem = RobotContainer.rollSubsystem;
    PitchSubsystem pitchSubsystem = RobotContainer.pitchSubsystem;
    VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;
    ExtenderSubsystem extenderSubsystem = RobotContainer.extenderSubsystem;
    IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    CommandXboxController driverXbox = RobotContainer.driverXbox;
    CommandXboxController manipulatorXbox = RobotContainer.manipulatorXbox;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond)*1.5; // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine pickupAndScoreAuto() {
        AutoRoutine routine = m_factory.newRoutine("taxi");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("test");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        //driveToMiddle.atTime(5).onTrue()

        return routine;
    }

    public AutoRoutine lineTest() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Line Test");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine_linetest.trajectory("LineTestCho");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine_linetest.active().onTrue(
            Commands.sequence(
                
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        driveToMiddle.atTime(2).onTrue(l2Position());
        driveToMiddle.atTime(3).onTrue(l3Position());
        driveToMiddle.atTime(3).onTrue(l4Position());
        driveToMiddle.atTime(4).onTrue(new Intake(intakeSubsystem, Mode.OUT));

        //driveToMiddle.atTime(5).onTrue()

        return routine_linetest;
    }

    public AutoRoutine leave() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Leave");

        AutoTrajectory driveToMiddle = routine_linetest.trajectory("Leave");

        routine_linetest.active().onTrue(
            Commands.sequence(
                
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        return routine_linetest;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    
/*     public Command visionDriveLeft() {
        return RobotContainer.visionDriveLeft();
    } */

/*     public Command visionDriveRight() {
        return drivetrain.applyRequest(() ->
        drive.withVelocityX((visionSubsystem.visionXDrive(driverXbox.getLeftY(), -0.05, false, true, visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDrive(-driverXbox.getLeftX(), 0.0, false, true, visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
            .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
    } */

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
}