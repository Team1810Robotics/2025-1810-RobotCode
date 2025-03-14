package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.WristConstants.PitchConstants;
import frc.robot.Constants.WristConstants.RollConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.RollSubsystem;

public class Auto extends Command {

  private ArmSubsystem armSubsystem;
  private ExtenderSubsystem extenderSubsystem;
  private PitchSubsystem pitchSubsystem;
  private RollSubsystem rollSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private AutoMode mode;

  private double armSetpoint, pitchSetpoint, rollSetpoint, extenderDistance;

  private boolean armFinished = false, extenderFinished = false, pitchFinished = false, rollFinished = false,
      intakeFinished = false;

  private double startTime;

  public enum AutoMode {
    l2, l3, l4,
    base, intake
  }

  public Auto(ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem, PitchSubsystem pitchSubsystem,
      RollSubsystem rollSubsystem, IntakeSubsystem intakeSubsystem, AutoMode mode) {
    this.armSubsystem = armSubsystem;
    this.extenderSubsystem = extenderSubsystem;
    this.pitchSubsystem = pitchSubsystem;
    this.rollSubsystem = rollSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.mode = mode;

    startTime = Timer.getFPGATimestamp();

    switch (mode) {
      case l2:
        armSetpoint = ArmConstants.L2_POSITION;
        pitchSetpoint = PitchConstants.L2_POSITION;
        rollSetpoint = RollConstants.L2_POSITION;
        extenderDistance = ExtenderConstants.L2_HEIGHT;
        break;
      case l3:
        armSetpoint = ArmConstants.L3_POSITION;
        pitchSetpoint = PitchConstants.L3_POSITION;
        rollSetpoint = RollConstants.L3_POSITION;
        extenderDistance = ExtenderConstants.L3_HEIGHT;
        break;
      case l4:
        armSetpoint = ArmConstants.L4_POSITION;
        pitchSetpoint = PitchConstants.L4_POSITION;
        rollSetpoint = RollConstants.L4_POSITION;
        extenderDistance = ExtenderConstants.L4_HEIGHT;
        break;
      case base:
        armSetpoint = ArmConstants.BASE_POSITION;
        pitchSetpoint = PitchConstants.BASE_POSITION;
        rollSetpoint = RollConstants.BASE_POSITION;
        extenderDistance = ExtenderConstants.BASE_HEIGHT;
        break;
      case intake:
        armSetpoint = ArmConstants.INTAKE_POSITION;
        pitchSetpoint = PitchConstants.INTAKE_POSITION;
        rollSetpoint = RollConstants.INTAKE_POSITION;
        extenderDistance = ExtenderConstants.BASE_HEIGHT;
        break;

      default:
        armSetpoint = armSubsystem.currentSetpoint;
        pitchSetpoint = pitchSubsystem.currentSetpoint;
        rollSetpoint = rollSubsystem.currentSetpoint;
        extenderDistance = extenderSubsystem.currentSetpoint;
        break;
    }

    addRequirements(armSubsystem, extenderSubsystem, pitchSubsystem, rollSubsystem, intakeSubsystem);
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    int blue = intakeSubsystem.getBlue();
    int distance = intakeSubsystem.getDistance();

    armSubsystem.run(armSetpoint);
    pitchSubsystem.run(pitchSetpoint);
    rollSubsystem.run(rollSetpoint);

    if (currentTime - startTime > 4) {
      extenderSubsystem.extend(extenderDistance);
    }

    armFinished = isClose(armSubsystem.getMeasurement(), armSetpoint, 5);
    extenderFinished = isClose(extenderSubsystem.getDistance(), extenderDistance, 0.1);
    pitchFinished = isClose(pitchSubsystem.getMeasurment(), pitchSetpoint, 3);
    rollFinished = isClose(rollSubsystem.getMeasurment(), rollSetpoint, 3);

    if (armFinished && extenderFinished && pitchFinished && rollFinished) {
      intakeFinished = isIntakeFinished();
      if (mode == AutoMode.intake && blue < 10 && distance > 1800) {
        intakeSubsystem.run(.1);
      } else if (mode == AutoMode.intake) {
        intakeSubsystem.run(1);
      } else {
        intakeSubsystem.run(-.25);
      }
      if (intakeFinished) {
        intakeSubsystem.stop();
      }
    }
  }

  public boolean isClose(double input, double target, double tolerance) {
    return Math.abs(input - target) <= tolerance;
  }

  private boolean isIntakeFinished() {
    int distance = intakeSubsystem.getDistance();
    int blue = intakeSubsystem.getBlue();

    if (mode == AutoMode.intake) {
      if (distance > 2000 && mode == AutoMode.intake && blue > 10) {
        return true;
      }
      return false;
    }

    if (distance < 1000 && mode != AutoMode.intake) {
      return true;
    }
    
    return false;
  }

  @Override
  public boolean isFinished() {
    return armFinished && extenderFinished && pitchFinished && rollFinished && intakeFinished;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
    extenderSubsystem.stop();
    intakeSubsystem.stop();
    pitchSubsystem.stop();
    rollSubsystem.stop();
  }
}
