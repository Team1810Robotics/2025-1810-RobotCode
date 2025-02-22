package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.RollConstants;

public class RollSubsystem extends SubsystemBase {

    private SparkMax rollMotor;
    private DutyCycleEncoder encoder;

    private PIDController rollpidController;

    public RollSubsystem() {
        rollMotor = new SparkMax(RollConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(RollConstants.ENCODER_ID);

        rollpidController = new PIDController(RollConstants.kP, RollConstants.kI, RollConstants.kD);

        Shuffleboard.getTab("Intake").addNumber("Roll Rad Raw", () -> encoder.get());
        Shuffleboard.getTab("Intake").addNumber("Roll Rad Adj", () -> encoder.get() - RollConstants.ENCODER_OFFSET);
        Shuffleboard.getTab("Intake").addNumber("Roll Deg", () -> getMeasurment());

        Shuffleboard.getTab("Intake").add("Roll PID", rollpidController);

        Shuffleboard.getTab("Encoder").addBoolean("Roll Encoder", () -> encoder.isConnected());
        Shuffleboard.getTab("Intake").addNumber("Poll PID Out", () -> -rollpidController.calculate(getMeasurment(), 0));
    }

    public double getMeasurment(){
      double position = encoder.get() - RollConstants.ENCODER_OFFSET;
      double degrees = position * 360;
     
      return degrees;
    }

    /**
     * Runs pitch motor with PID
     * @param setPoint setpoint for wrist
     */
    public void run(double setpoint) {
      if(encoder.isConnected()) rollMotor.set(rollpidController.calculate(getMeasurment(), setpoint));
      else stop();
    }

    public void stop() {
        rollMotor.stopMotor();
    }
    
}