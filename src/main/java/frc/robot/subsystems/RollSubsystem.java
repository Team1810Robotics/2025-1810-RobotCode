package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    }

    public void runRoll(double setPoint) {
        rollMotor.set(rollpidController.calculate(getPostiion(), setPoint));
    }

    public double getPostiion() {
        return (encoder.get() - RollConstants.ENCODER_OFFSET)  * 360; 
    }

    public boolean atSetPoint(double setpoint) {
        return rollpidController.atSetpoint();
    }

    public void stop() {
        rollMotor.stopMotor();
    }
    
}
