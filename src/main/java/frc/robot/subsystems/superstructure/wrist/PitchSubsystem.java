package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configs;
import frc.robot.util.constants.RobotConstants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {

  private TalonFX pitchMotor;

  private PIDController pitchPIDController;

  public double currentSetpoint = 0;
  
  public PitchSubsystem() {
    pitchMotor = new TalonFX(PitchConstants.MOTOR_ID);

    pitchPIDController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);
    pitchPIDController.setTolerance(PitchConstants.TOLERANCE);

    pitchMotor.getConfigurator().apply(Configs.getPitchConfig());

    Shuffleboard.getTab("Pitch").addNumber("Pitch Deg", () -> getMeasurement());
    Shuffleboard.getTab("Pitch").add("Pitch PID", pitchPIDController);
    Shuffleboard.getTab("Pitch").addNumber("Current Setpoint", () -> currentSetpoint);
    Shuffleboard.getTab("Pitch").addNumber("Pitch PID Out", () -> pitchPIDController.calculate(getMeasurement(), currentSetpoint));
    Shuffleboard.getTab("Pitch").addNumber("Pitch Raw", () -> pitchMotor.getPosition().getValueAsDouble());
    Shuffleboard.getTab("Pitch").addNumber("Pitch Raw Adj", () -> pitchMotor.getPosition().getValueAsDouble() - PitchConstants.ENCODER_OFFSET);
  }

  /**
   * Returns the current measurement of the pitch motor in degrees.
   *
   * <p>
   * This method reads the current position of the pitch motor's encoder, subtracts the
   * {@link PitchConstants#ENCODER_OFFSET} to center the measurement, and then converts
   * it to degrees using {@link Units#rotationsToDegrees(double)}.
   *
   * @return the current measurement of the pitch motor in degrees
   */
  public double getMeasurement() {
    double position = pitchMotor.getPosition().getValueAsDouble() - PitchConstants.ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return -degrees;
  }

  public boolean atSetpoint() {
    return pitchPIDController.atSetpoint();
  }

  public void stop() {
    pitchMotor.stopMotor();
  }

  /**
   * Runs the pitch motor with PID
   * 
   * @param setPoint Setpoint for wrist
   * @return Command to run the motor
   */
  public Command run(double setPoint) {
    return Commands.run(() -> pitchMotor.set(MathUtil.clamp(pitchPIDController.calculate(getMeasurement(), setPoint), 0, .3)), this).finallyDo(() -> stop());
  }

}