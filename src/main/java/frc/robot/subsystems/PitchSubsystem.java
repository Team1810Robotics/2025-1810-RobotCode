package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {
    //private SparkMax pitchMotor;
    private DutyCycleEncoder encoder;

    private SparkMaxConfig config;

    private TalonFX pitchMotor;

    private PIDController pitchPIDController;

    public PitchSubsystem() {
        //pitchMotor = new SparkMax(PitchConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        pitchMotor = new TalonFX(PitchConstants.MOTOR_ID);
        encoder = new DutyCycleEncoder(PitchConstants.ENCODER_ID);

        config = new SparkMaxConfig();
        config.smartCurrentLimit(40);

        pitchPIDController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);

        Shuffleboard.getTab("Intake").addNumber("Pitch Rad Adjust", () -> encoder.get() - PitchConstants.ENCODER_OFFSET);
        Shuffleboard.getTab("Intake").addNumber("Pitch Rad Raw", () -> encoder.get());
        Shuffleboard.getTab("Intake").addNumber("Pitch Deg", () -> getMeasurment());
        Shuffleboard.getTab("Intake").addNumber("PitchOut", () -> pitchPIDController.calculate(getMeasurment(), 0));

        Shuffleboard.getTab("Intake").add("Pitch PID", pitchPIDController);

        Shuffleboard.getTab("Encoder").addBoolean("Pitch Encoder", () -> encoder.isConnected());

        Shuffleboard.getTab("Intake").addNumber("Setpoint", () -> pitchPIDController.getSetpoint());
    }

    public double getMeasurment(){
      double position = encoder.get() - PitchConstants.ENCODER_OFFSET;
      double degrees = Units.rotationsToDegrees(position);
     
      return degrees; 
    }

    /**
     * Runs the pitch motor with PID
     * @param setPoint Setpoint for wrist
     */
    public void run(double setPoint) {
      if (encoder.isConnected()) {
        pitchMotor.set(pitchPIDController.calculate(getMeasurment(), setPoint));
      }
      else {
        System.out.println("Pitch Encoder Disconnected");
        stop();
        pitchMotor.disable();
      }
    }

    public void stop() {
        pitchMotor.stopMotor();
    }
    
}