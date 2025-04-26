package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {

  private TalonFX pitchMotor;
  private DutyCycleEncoder encoder;

  private PIDController pitchPIDController;

  public boolean badEncoder = false;

  public PitchSubsystem() {
    pitchMotor = new TalonFX(PitchConstants.MOTOR_ID);
    encoder = new DutyCycleEncoder(PitchConstants.ENCODER_ID);

    pitchPIDController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);

    Shuffleboard.getTab("Intake").addNumber("Pitch Rad Adjust", () -> encoder.get() - PitchConstants.ENCODER_OFFSET);
    Shuffleboard.getTab("Intake").addNumber("Pitch Rad Raw", () -> encoder.get());
    Shuffleboard.getTab("Intake").addNumber("Pitch Deg", () -> getMeasurment());
    Shuffleboard.getTab("Intake").addNumber("PitchOut", () -> pitchPIDController.calculate(getMeasurment(), 0));

    Shuffleboard.getTab("Intake").add("Pitch PID", pitchPIDController);

    Shuffleboard.getTab("Encoder").addBoolean("Pitch Encoder", () -> encoder.isConnected());

  }

  public double getMeasurment() {
    double position = encoder.get() - PitchConstants.ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return degrees;
  }

  public boolean isEncoderConnected() {
    return encoder.isConnected();
  }

  public static double clamp(double value) {
    return Math.max(-.3, Math.min(value, 0.3));
  }

  /**
   * Runs the pitch motor with PID
   * 
   * @param setPoint Setpoint for wrist
   * @return Command to run the motor
   */
  public Command run(double setPoint) {
    if (encoder.isConnected() && !badEncoder) {
      return Commands.run(() -> pitchMotor.set(clamp(pitchPIDController.calculate(getMeasurment(), setPoint))), this).finallyDo(() -> stop());
    } else {
      badEncoder = true;
      System.out.println("Pitch Encoder Disconnected");
      stop();
      pitchMotor.disable();
      return new InstantCommand();
    }
  }

  public boolean atSetpoint() {
    return Math.abs(getMeasurment() - pitchPIDController.getSetpoint()) < PitchConstants.TOLERANCE;
  }

  public void stop() {
    pitchMotor.stopMotor();
  }

}