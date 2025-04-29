package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.constants.RobotConstants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {

  private TalonFX pitchMotor;

  private PIDController pitchPIDController;
  
  public PitchSubsystem() {
    pitchMotor = new TalonFX(PitchConstants.MOTOR_ID);

    pitchPIDController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);
    pitchPIDController.setTolerance(PitchConstants.TOLERANCE);

    pitchMotor.getConfigurator().apply(Config.getPitchConfig());

    Shuffleboard.getTab("Intake").addNumber("Pitch Deg", () -> getMeasurment());
    Shuffleboard.getTab("Intake").add("Pitch PID", pitchPIDController);
  }

  public double getMeasurment() {
    double position = pitchMotor.getPosition().getValueAsDouble() / 16384 - PitchConstants.ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return degrees;
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
    return Commands.run(() -> pitchMotor.set(MathUtil.clamp(pitchPIDController.calculate(getMeasurment(), setPoint), 0, .3)), this).finallyDo(() -> stop());
  }

}