package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

    private SparkMax extenderMotor;
    private DutyCycleEncoder encoder;
    private SparkMaxConfig config;

    private PIDController extenderPIDController;

    private double cumulativeRotations = 0;
    private double previousRotation = 0;


    public ExtenderSubsystem() {
        extenderMotor = new SparkMax(ExtenderConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(ExtenderConstants.ENCODER_ID);

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        extenderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD); //I have no clue if this will work

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder Raw", () -> encoder.get());
        Shuffleboard.getTab("Extender").addNumber("Extender Encoder Adj", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getDistance());

        Shuffleboard.getTab("Extender").addNumber("Cumulative Rotations", () -> cumulativeRotations);

        Shuffleboard.getTab("Extender").add("Extender PID", extenderPIDController);

        Shuffleboard.getTab("Extender").addNumber("Motor Power", () -> extenderMotor.getAppliedOutput());

        Shuffleboard.getTab("Extender").addBoolean("Extender Encoder", () -> encoder.isConnected());
    }

    /**
     * Get the total number of rotations the extender has gone through.
     *
     * <p>This method detects when the encoder wraps around and adds one to the cumulative
     * count. This is necessary because the DutyCycleEncoder wraps around to 0 after a certain
     * point, so simply adding up the positions would not give an accurate total.
     *
     * @return the total number of rotations the extender has gone through
     */
    
     public void totalRotations() {
        double currentRotation = getEncoder();
        
        // Calculate the delta, accounting for wraparound
        double delta = currentRotation - previousRotation;
        
        // Adjust for wraparound cases
        if (delta > 0.5) {
            delta -= 1.0; // Wrapped from 1.0 to 0.0 (moving backward)
        } else if (delta < -0.5) {
            delta += 1.0; // Wrapped from 0.0 to 1.0 (moving forward)
        }
        
        // Update cumulative position without relying on fullRotations counter
        cumulativeRotations -= delta; // Keep negative sign if needed for direction
        
        // Store for next calculation
        previousRotation = currentRotation;
    }

    public double getEncoder() {
        return encoder.get() - ExtenderConstants.ENCODER_OFFSET;
    }

    public void run(double speed) {
        if (encoder.isConnected()){
            extenderMotor.set(speed);
        }
    }

    /**
     * Returns the total distance the extender has extended, in inches.
     *
     * <p>This method uses the total number of rotations the encoder has gone through,
     * as calculated by the {@link #totalRotations()} method.
     *
     * @return the total distance the extender has extended, in inches
     */
    public double getDistance() {
        return cumulativeRotations * ExtenderConstants.INCHES_PER_ROTATION;
    }

    public void extend(double height) {
        if (encoder.isConnected()) {        
            extenderMotor.set(extenderPIDController.calculate(getDistance(), height));
        }
    }

    
    public void stop() {
        extenderMotor.stopMotor();
    }
    
    @Override
    public void periodic() {
        totalRotations();
    }

    

}