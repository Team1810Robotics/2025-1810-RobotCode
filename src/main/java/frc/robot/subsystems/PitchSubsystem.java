package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {
    private SparkMax pitchMotor;
    private DutyCycleEncoder encoder;

    private PIDController pitchpidController;

    public PitchSubsystem() {
        pitchMotor = new SparkMax(PitchConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(PitchConstants.ENCODER_ID);

        pitchpidController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);

        Shuffleboard.getTab("Intake").addNumber("Pitch Rad Adjust", () -> encoder.get() - PitchConstants.ENCODER_OFFSET);
        Shuffleboard.getTab("Intake").addNumber("Pitch Rad Raw", () -> encoder.get());
        Shuffleboard.getTab("Intake").addNumber("Pitch Deg", () -> getMeasurment());
        Shuffleboard.getTab("Intake").addNumber("PitchOut", () -> pitchpidController.calculate(getMeasurment(), 0));

        Shuffleboard.getTab("Intake").add("Pitch PID", pitchpidController);

        Shuffleboard.getTab("Encoder").addBoolean("Pitch Encoder", () -> encoder.isConnected());
    }

    public double getMeasurment(){
      double position = encoder.get() - PitchConstants.ENCODER_OFFSET;
      double degrees = position * 360;
     
      return degrees; 
    }

    /**
     * Runs the pitch motor with PID
     * @param setPoint Setpoint for wrist
     */
    public void run(double setPoint) {
      if (encoder.isConnected())  pitchMotor.set(pitchpidController.calculate(getMeasurment(), setPoint));
      else stop();
    }



    public void stop() {
        pitchMotor.stopMotor();
    }
    
}