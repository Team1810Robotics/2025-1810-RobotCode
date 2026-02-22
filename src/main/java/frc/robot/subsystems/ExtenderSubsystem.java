package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.util.Configs;
import frc.robot.util.ShuffleboardTabs;

public class ExtenderSubsystem extends SubsystemBase {

    private TalonFX extenderMotor;
    private DutyCycleEncoder encoder;

    private PIDController extenderPIDController;

    private double cumulativeRotations = 0;
    private double previousRotation = 0;

    public double currentSetpoint;

    public DigitalInput limitSwitch;

    private final ShuffleboardTab tab = ShuffleboardTabs.EXTENDER;

    public ExtenderSubsystem() {
        extenderMotor = new TalonFX(ExtenderConstants.MOTOR_ID);
        encoder = new DutyCycleEncoder(ExtenderConstants.ENCODER_ID);

        limitSwitch = new DigitalInput(4);

        extenderMotor.getConfigurator().apply(Configs.getExtenderConfig());

        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD);

        tab.addNumber("Inches", () -> getDistance());
        tab.addNumber("Raw", () -> encoder.get());
        tab.addNumber("PID Out", () -> extenderPIDController.calculate(getDistance(), currentSetpoint));
        tab.add("PID", extenderPIDController);
        tab.addBoolean("Endstop", () -> getLimitSwitch());
    }

    public boolean isEncoderConnected() {
        return encoder.isConnected();
    }

    /**
     * Get the total number of rotations the extender has gone through.
     *
     * <p>
     * This method detects when the encoder wraps \around and adds one to the
     * cumulative
     * count. This is necessary because the DutyCycleEncoder wraps around to 0 after
     * a certain
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
        if (encoder.isConnected()) {
            extenderMotor.set(speed);
        }
    }

    /**
     * Returns the total distance the extender has extended, in inches.
     *
     * <p>
     * This method uses the total number of rotations the encoder has gone through,
     * as calculated by the {@link #totalRotations()} method.
     *
     * @return the total distance the extender has extended, in inches
     */
    public double getDistance() {
        return cumulativeRotations * ExtenderConstants.INCHES_PER_ROTATION;
    }

    public void extend(double height) {
        currentSetpoint = height;
        if (encoder.isConnected()) {
            extenderMotor.set(extenderPIDController.calculate(getDistance(), height));
        } else {
            stop();
            extenderMotor.disable();
            System.out.println("Extender Encoder Disconnected");
        }
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void reset() {
        cumulativeRotations = 0;
        previousRotation = 0;
        ExtenderConstants.ENCODER_OFFSET = encoder.get();
    }

    public void stop() {
        extenderMotor.stopMotor();
    }

    @Override
    public void periodic() {
        totalRotations();
    }
}