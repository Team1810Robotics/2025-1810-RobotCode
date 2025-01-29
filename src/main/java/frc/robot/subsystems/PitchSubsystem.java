package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    }

    public void runPitch(double setPoint) {
        pitchMotor.set(pitchpidController.calculate(getPostiion(), setPoint));
    }

    public double getPostiion() {
        return (encoder.get() - PitchConstants.ENCODER_OFFSET) * 360; 
    }

    public boolean atSetPoint(double setpoint) {
        return pitchpidController.atSetpoint();
    }

    public void stop() {
        pitchMotor.stopMotor();
    }
    
}
