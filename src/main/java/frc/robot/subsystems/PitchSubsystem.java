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

        Shuffleboard.getTab("Intake").addNumber("Pitch Rad", () -> encoder.get());
        Shuffleboard.getTab("Intake").addNumber("Pitch Deg", () -> getMeasurment());

        Shuffleboard.getTab("Intake").add("Pitch PID", pitchpidController);
    }

    public double getMeasurment(){
      double position = encoder.get() -0.9;
      double degrees = position * 360;
     
      return degrees; //TODO: change zero to real offset
    }

    /**
     * Runs the pitch motor with PID
     * @param setPoint Setpoint for wrist
     */
    public void runPitch(double setPoint) {
        pitchMotor.set(pitchpidController.calculate(getMeasurment(), setPoint));
    }

    public void run(double speed) {
        pitchMotor.set(speed);
    }

    public boolean atSetPoint(double setpoint) {
        return pitchpidController.atSetpoint();
    }

    public void stop() {
        pitchMotor.stopMotor();
    }
    
}