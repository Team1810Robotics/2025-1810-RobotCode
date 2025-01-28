package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.RollConstants;

public class RollSubsystem extends SubsystemBase {

    private SparkMax rollMotor;
    private AbsoluteEncoder encoder;

    private PIDController rollpidController;
    public RollSubsystem() {
        rollMotor = new SparkMax(RollConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        encoder = rollMotor.getAbsoluteEncoder();

        rollpidController = new PIDController(RollConstants.kP, RollConstants.kI, RollConstants.kD);
    }

    public void runRoll(double setPoint) {
        rollMotor.set(rollpidController.calculate(getPostiion(), setPoint));
    }

    public double getPostiion() {
        return encoder.getPosition() * 360; //TODO: Figure out offset
    }

    public boolean atSetPoint(double setpoint) {
        return rollpidController.atSetpoint();
    }

    public void stop() {
        rollMotor.stopMotor();
    }
    
}
