package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configs;
import frc.robot.util.constants.RobotConstants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {

  private TalonFX pitchMotor;
  private double setpoint = 0;
  
  public PitchSubsystem() {
    pitchMotor = new TalonFX(PitchConstants.MOTOR_ID);


    pitchMotor.getConfigurator().apply(Configs.getPitchConfig());

    Shuffleboard.getTab("Intake").addNumber("Pitch Deg", () -> getMeasurment());
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
  public double getMeasurment() {
    double position = pitchMotor.getPosition().getValueAsDouble() / 16384 - PitchConstants.ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return degrees;
  }

  public boolean atSetpoint() {
    return Math.abs(getMeasurment() - setpoint) < PitchConstants.TOLERANCE;
  }

  public void stop() {
    pitchMotor.stopMotor();
  }

  /**
   * Runs the pitch motor with PID
   * 
   * @param setpoint Setpoint for wrist
   * @return Command to run the motor
   */
  public Command run(double setpoint) {
    this.setpoint = setpoint;

    MotionMagicVoltage request = new MotionMagicVoltage(setpoint);
    return Commands.run(() -> pitchMotor.setControl(request), this).finallyDo(() -> stop());
  }

}