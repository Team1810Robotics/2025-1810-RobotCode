package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.RollConstants;

public class RollSubsystem extends SubsystemBase {

    private SparkMax rollMotor;
    private DutyCycleEncoder encoder;

    private PIDController rollPIDController;

    private SparkMaxConfig config;

    public RollSubsystem() {
        rollMotor = new SparkMax(RollConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(RollConstants.ENCODER_ID);

        rollPIDController = new PIDController(RollConstants.kP, RollConstants.kI, RollConstants.kD);

        config = new SparkMaxConfig();
        config.smartCurrentLimit(40);

        rollMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        Shuffleboard.getTab("Intake").addNumber("Roll Rad Raw", () -> encoder.get());
        Shuffleboard.getTab("Intake").addNumber("Roll Rad Adj", () -> encoder.get() - RollConstants.ENCODER_OFFSET);
        Shuffleboard.getTab("Intake").addNumber("Roll Deg", () -> getMeasurment());

        Shuffleboard.getTab("Intake").add("Roll PID", rollPIDController);

        Shuffleboard.getTab("Encoder").addBoolean("Roll Encoder", () -> encoder.isConnected());
        Shuffleboard.getTab("Intake").addNumber("Poll PID Out", () -> -rollPIDController.calculate(getMeasurment(), 0));
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
      if(encoder.isConnected()) rollMotor.set(rollPIDController.calculate(getMeasurment(), setpoint));
      else stop();
    }

    public void stop() {
        rollMotor.stopMotor();
    }
    
}