package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.RollConstants;
import frc.robot.util.Configs;
import frc.robot.util.ShuffleboardTabs;

public class RollSubsystem extends SubsystemBase {

  private SparkMax rollMotor;
  private DutyCycleEncoder encoder;

  public PIDController rollPIDControllerIncreasing;
  public PIDController rollPIDControllerDecreasing;

  private SparkMaxConfig config;

  public double currentSetpoint;

  private final ShuffleboardTab tab = ShuffleboardTabs.ROLL;

  public RollSubsystem() {
    rollMotor = new SparkMax(RollConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
    encoder = new DutyCycleEncoder(RollConstants.ENCODER_ID);

    rollPIDControllerIncreasing = new PIDController(RollConstants.kPI, RollConstants.kII, RollConstants.kDI);
    rollPIDControllerDecreasing = new PIDController(RollConstants.kPD, RollConstants.kID, RollConstants.kDD);

    config = Configs.getRollConfig();

    rollMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    tab.addNumber("Degree",() -> getMeasurment());
    tab.addNumber("Raw", () -> encoder.get());
    tab.addNumber("Raw Adjusted", () -> encoder.get() - RollConstants.ENCODER_OFFSET);
    tab.addNumber("PID Increasing Out", () -> rollPIDControllerIncreasing.calculate(getMeasurment(), currentSetpoint));
    tab.addNumber("PID Decreasing Out", () -> rollPIDControllerDecreasing.calculate(getMeasurment(), currentSetpoint));
    tab.add("PID Increasing", rollPIDControllerIncreasing);
    tab.add("PID Decreasing", rollPIDControllerDecreasing);

  }

  public double getMeasurment() {
    double position = encoder.get() - RollConstants.ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return degrees;
  }

  public boolean isEncoderConnected() {
    return encoder.isConnected();
  }

  /**
   * Runs pitch motor with PID
   * 
   * @param setPoint setpoint for wrist
   */
  public void run(double setpoint) {
    if (encoder.isConnected()) {
      currentSetpoint = setpoint;
      if (getMeasurment() < setpoint) {
        // System.out.println("Rolling to: " + setpoint + " L:" + getMeasurment());
        rollMotor.set(rollPIDControllerIncreasing.calculate(getMeasurment(), setpoint));
      } else if (getMeasurment() > setpoint) {
        // System.out.println("Rolling to: " + setpoint + " L:" + getMeasurment());
        rollMotor.set(rollPIDControllerDecreasing.calculate(getMeasurment(), setpoint));
      }
    } else {
      System.out.println("Roll Encoder Disconnected");
      stop();
      rollMotor.disable();
    }
  }

  public void runManual(double speed) {
    rollMotor.set(speed);
  }

  public void stop() {
    rollMotor.stopMotor();
  }

}