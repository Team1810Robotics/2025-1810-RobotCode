package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.commands.Extender;

public class ExtenderSubsystem extends SubsystemBase {

    private SparkMax extenderMotor;
    private DutyCycleEncoder encoder;

    private PIDController extenderpidController;

    public ExtenderSubsystem() {
        extenderMotor = new SparkMax(ExtenderConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(ExtenderConstants.ENCODER_ID, 18.0 / ExtenderConstants.INCHES_PER_ROTATION, 0);

        extenderpidController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD); //I have no clue how this would work
    }


    public double getRotations() {
        return encoder.get();
    }

    public double getDistance() {
        return getRotations() * ExtenderConstants.INCHES_PER_ROTATION;
    }

    public void runExtender(double speed) {
        extenderMotor.set(speed);
    }
    
    public void stop() {
        extenderMotor.stopMotor();
    }
    

}
