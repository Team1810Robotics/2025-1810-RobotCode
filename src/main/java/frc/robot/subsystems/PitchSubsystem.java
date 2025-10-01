package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.PitchConstants;
import frc.robot.util.Configs;
import frc.robot.util.ShuffleboardTabs;

public class PitchSubsystem extends SubsystemBase {
  private TalonFX pitchMotor;

  private PIDController pitchPIDController;

  public double currentSetpoint;

  private final ShuffleboardTab tab = ShuffleboardTabs.PITCH;


  public PitchSubsystem() {
    pitchMotor = new TalonFX(PitchConstants.MOTOR_ID);

    pitchMotor.getConfigurator().apply(Configs.getPitchConfig());

    pitchPIDController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);

    tab.addNumber("Degree",() -> getMeasurment());
    tab.addNumber("Raw", () -> pitchMotor.getPosition().getValueAsDouble());
    tab.addNumber("Raw Adjusted", () -> pitchMotor.getPosition().getValueAsDouble() - PitchConstants.ENCODER_OFFSET);
    tab.addNumber("PID Out", () -> pitchPIDController.calculate(getMeasurment(), currentSetpoint));
    tab.add("PID", pitchPIDController);
  }

  public double getMeasurment() {
    double position = pitchMotor.getPosition().getValueAsDouble() - PitchConstants.ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return -degrees;
  }

  /**
   * Runs the pitch motor with PID
   * 
   * @param setpoint Setpoint for wrist
   */
  public void run(double setpoint) {
    currentSetpoint = setpoint;

    pitchMotor.set(MathUtil.clamp(-
    pitchPIDController.calculate(getMeasurment(), setpoint), -.3, .3));
  }

  public void stop() {
    pitchMotor.stopMotor();
  }

}