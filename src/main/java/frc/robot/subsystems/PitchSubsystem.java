package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.PitchConstants;

public class PitchSubsystem extends SubsystemBase {
    private SparkMax pitchMotor;
    private AbsoluteEncoder encoder;

    private PIDController pitchpidController;

    public PitchSubsystem() {
        pitchMotor = new SparkMax(PitchConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        encoder = pitchMotor.getAbsoluteEncoder();

        pitchpidController = new PIDController(PitchConstants.kP, PitchConstants.kI, PitchConstants.kD);
    }

    public void runPitch(double setPoint) {
        pitchMotor.set(pitchpidController.calculate(getPostiion(), setPoint));
    }

    public double getPostiion() {
        return encoder.getPosition() * 360; //TODO: Figure out offset
    }

    public boolean atSetPoint(double setpoint) {
        return pitchpidController.atSetpoint();
    }

    public void stop() {
        pitchMotor.stopMotor();
    }
    
}
