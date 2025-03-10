package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    public final AutoFactory m_factory;
    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    RollSubsystem rollSubsystem = RobotContainer.rollSubsystem;
    PitchSubsystem pitchSubsystem = RobotContainer.pitchSubsystem;
    VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;
    ExtenderSubsystem extenderSubsystem = RobotContainer.extenderSubsystem;
    IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    CommandXboxController driverXbox = RobotContainer.driverXbox;
    CommandXboxController manipulatorXbox = RobotContainer.manipulatorXbox;

    CommandSwerveDrivetrain drivetrain;

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond)*1.5; // 3/4 of a rotation per second max angular velocity

    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        m_factory = factory;
    }

    public AutoRoutine lineTest() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Line Test");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine_linetest.trajectory("LineTestCho");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine_linetest.active().onTrue(
            Commands.sequence(
                //new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)),
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        //driveToMiddle.done().onTrue(new Arm(armSubsystem, ArmConstants.L3_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L3_POSITION), new Pitch(pitchSubsystem, PitchConstants.L3_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT)));;
        driveToMiddle.done(30).onTrue(new Arm(armSubsystem, ArmConstants.L4_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L4_POSITION), new Pitch(pitchSubsystem, PitchConstants.L4_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L4_HEIGHT)));
        driveToMiddle.done(120).onTrue(new Intake(intakeSubsystem, Mode.OUT));
        driveToMiddle.done(150).onTrue(new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)));
        driveToMiddle.done(200).onTrue(new Intake(intakeSubsystem, Mode.STOP));

        //driveToMiddle.atTime(5).onTrue()

        return routine_linetest;
    }

    public AutoRoutine proper() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Proper Fac");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine_linetest.trajectory("Proper");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine_linetest.active().onTrue(
            Commands.sequence(
                //new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)),
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        //driveToMiddle.done().onTrue(new Arm(armSubsystem, ArmConstants.L3_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L3_POSITION), new Pitch(pitchSubsystem, PitchConstants.L3_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT)));;
        // driveToMiddle.done(30).onTrue(new Arm(armSubsystem, ArmConstants.L4_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L4_POSITION), new Pitch(pitchSubsystem, PitchConstants.L4_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L4_HEIGHT)));
        // driveToMiddle.done(120).onTrue(new Intake(intakeSubsystem, Mode.OUT));
        // driveToMiddle.done(150).onTrue(new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)));
        // driveToMiddle.done(200).onTrue(new Intake(intakeSubsystem, Mode.STOP));

        //driveToMiddle.atTime(5).onTrue()

        return routine_linetest;
    }

    public AutoRoutine dis() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Dis Fac");

        AutoTrajectory driveToMiddle = routine_linetest.trajectory("DisTest");

        routine_linetest.active().onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        return routine_linetest;
    }

    public AutoRoutine visionTest() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Dis Fac");

        AutoTrajectory vistest = routine_linetest.trajectory("VisionTest");
        AutoTrajectory vistest2 = routine_linetest.trajectory("VisionTest");

        routine_linetest.active().onTrue(
            Commands.sequence(
                vistest.resetOdometry(),
                vistest.cmd()
            )
        );

        vistest.done(50).whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.05, true, visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
            .withRotationalRate(0)));
        vistest.done(100).onTrue(new Intake(intakeSubsystem, Mode.OUT));
        vistest.done(200).onTrue(new Intake(intakeSubsystem, Mode.STOP));
        vistest.done(201).onTrue(vistest2.resetOdometry().alongWith(vistest2.cmd()));

        return routine_linetest;
    }

    public AutoRoutine lineTestStraight() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Reef Straight");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine_linetest.trajectory("LineTestChoSt");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine_linetest.active().onTrue(
            Commands.sequence(
                //new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)),
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        //driveToMiddle.done().onTrue(new Arm(armSubsystem, ArmConstants.L3_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L3_POSITION), new Pitch(pitchSubsystem, PitchConstants.L3_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L3_HEIGHT)));;
        driveToMiddle.done(100).onTrue(new Arm(armSubsystem, ArmConstants.L4_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.L4_POSITION), new Pitch(pitchSubsystem, PitchConstants.L4_POSITION), new Extender(extenderSubsystem, ExtenderConstants.L4_HEIGHT)));
        driveToMiddle.done(190).onTrue(new Intake(intakeSubsystem, Mode.OUT));
        driveToMiddle.done(300).onTrue(new Arm(armSubsystem, ArmConstants.BASE_POSITION).alongWith(new Roll(rollSubsystem, RollConstants.BASE_POSITION), new Pitch(pitchSubsystem, PitchConstants.BASE_POSITION), new Extender(extenderSubsystem, ExtenderConstants.BASE_HEIGHT)));
        driveToMiddle.done(300).onTrue(new Intake(intakeSubsystem, Mode.STOP));

        //driveToMiddle.atTime(5).onTrue()

        return routine_linetest;
    }

    public AutoRoutine move() {
        AutoRoutine routine_linetest = m_factory.newRoutine("Super Leave");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine_linetest.trajectory("BackLeftReefStart");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine_linetest.active().onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );

        //driveToMiddle.atTime(5).onTrue()

        return routine_linetest;
    }

    public AutoRoutine leave() {
        AutoRoutine routine_leave = m_factory.newRoutine("Leave");

        AutoTrajectory leaveTrj = routine_leave.trajectory("Leave");

        routine_leave.active().onTrue(
            Commands.sequence(
                leaveTrj.resetOdometry(),
                leaveTrj.cmd(),
                Commands.print("Running Leave Auto")
            )
        );

        return routine_leave;
    }

    public Command visionDriveRight() {
        return drivetrain.applyRequest(() ->
        drive.withVelocityX((visionSubsystem.visionXDriveLeft(driverXbox.getLeftY(), -0.05, true, visionSubsystem.driveControllerY) * MaxSpeed) / 3.5) // Drive forward with negative Y (forward)
            .withVelocityY((-visionSubsystem.visionYDriveLeft(-driverXbox.getLeftX(), 0.0, true, visionSubsystem.driveControllerX) * MaxSpeed) / 3.5) // Drive left with negative X (left)
            .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
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
}