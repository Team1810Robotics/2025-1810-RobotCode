package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

    private TalonFX extenderMotor;
    private DutyCycleEncoder encoder;

    private TalonFXConfigurator configuration;
    private CurrentLimitsConfigs currentLimitsConfigs;

    //private SparkMax extenderMotor;
    //private SparkMaxConfig config;

    private PIDController extenderPIDController;

    private double cumulativeRotations = 0;
    private double previousRotation = 0;


    public ExtenderSubsystem() {
        extenderMotor = new TalonFX(ExtenderConstants.MOTOR_ID);
        //extenderMotor = new SparkMax(ExtenderConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(ExtenderConstants.ENCODER_ID);

        //config = new SparkMaxConfig();

        //config.smartCurrentLimit(40);
        //config.idleMode(IdleMode.kBrake);
        //extenderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configuration = extenderMotor.getConfigurator();

        currentLimitsConfigs.StatorCurrentLimit = 40;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        configuration.apply(currentLimitsConfigs);
        
        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD);

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder Raw", () -> encoder.get());
        Shuffleboard.getTab("Extender").addNumber("Extender Encoder Adj", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getDistance());

        Shuffleboard.getTab("Extender").addNumber("Cumulative Rotations", () -> cumulativeRotations);

        Shuffleboard.getTab("Extender").add("Extender PID", extenderPIDController);

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
        } else {
            stop();
            extenderMotor.disable();
            System.out.println("Extender Encoder Disconnected");
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