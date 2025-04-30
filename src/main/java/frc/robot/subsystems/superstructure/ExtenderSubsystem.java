package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

    private TalonFX extenderMotor;
    private DutyCycleEncoder encoder;

    private TalonFXConfigurator configuration;
    private CurrentLimitsConfigs currentLimitsConfigs;

    private final PIDController extenderPIDController;

    private double cumulativeRotations = 0;
    private double previousRotation = 0;

    private double currentSetpoint;

    private DigitalInput limitSwitch;

    public ExtenderSubsystem() {
        extenderMotor = new TalonFX(ExtenderConstants.MOTOR_ID);
        encoder = new DutyCycleEncoder(ExtenderConstants.ENCODER_ID);

        limitSwitch = new DigitalInput(ExtenderConstants.LIMIT_SWITCH_ID);

        configuration = extenderMotor.getConfigurator();
        currentLimitsConfigs = new CurrentLimitsConfigs();

        currentLimitsConfigs.StatorCurrentLimit = 45;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        configuration.apply(currentLimitsConfigs);

        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD);

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder Raw", () -> encoder.get());
        Shuffleboard.getTab("Extender").addNumber("Extender Encoder Adj", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getDistance());

        Shuffleboard.getTab("Extender").addNumber("Cumulative Rotations", () -> cumulativeRotations);

        Shuffleboard.getTab("Extender").add("Extender PID", extenderPIDController);

        Shuffleboard.getTab("Extender").addBoolean("Extender Encoder", () -> encoder.isConnected());

        Shuffleboard.getTab("Extender").addBoolean("Endstop", () -> !limitSwitch.get());
    }

    public boolean isEncoderConnected() {
        return encoder.isConnected();
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public double getEncoder() {
        return encoder.get() - ExtenderConstants.ENCODER_OFFSET;
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void stop() {
        extenderMotor.stopMotor();
    }

    /**
     * Get the total number of rotations the extender has gone through.
     *
     * <p>
     * This method detects when the encoder wraps around and adds one to the
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

    /**
     * Resets the extender subsystem to its starting state.
     *
     * <p>
     * This method resets the cumulative rotations and previous rotation values to 0, and
     * sets the encoder offset to the current encoder value.
     */
    public void reset() {
        cumulativeRotations = 0;
        previousRotation = 0;
        ExtenderConstants.ENCODER_OFFSET = encoder.get();
    }

    /**
     * Gets the current extension of the extender as a percentage.
     *
     * <p>
     * This method returns the current extension of the extender as a value between 0 and 1,
     * where 0 is the minimum extension and 1 is the maximum extension.
     *
     * @return the current extension of the extender as a percentage
     */
    public double getExtensionPercent() {
        double percent = (getDistance() - ExtenderConstants.MIN_EXTENSION) / (ExtenderConstants.MAX_EXTENSION - ExtenderConstants.MIN_EXTENSION);
        return Math.max(0, Math.min(percent, 1));
    }

    /**
     * Runs the extender motor manually.
     *
     * <p>
     * This method sets the speed of the extender motor to the specified value.
     *
     * @param speed the speed to set the extender motor to
     * @return a command that runs the extender motor manually
     * @see #run(double)
     */
    public Command runManual(double speed) {
        if (encoder.isConnected()) {
            return Commands.run(() -> extenderMotor.set(speed), this);
        }
        return new InstantCommand();
    }


    /**
     * Runs the extender motor with PID control.
     *
     * <p>
     * This method uses the current setpoint and the encoder value to calculate the
     * output for the motor.
     *
     * @param height the desired height in inches
     * @return a command that runs the extender motor with PID control
     * @see #runManual(double)
     */
    public Command run(double height) {
        currentSetpoint = height;
        if (encoder.isConnected()) {
            return Commands.run(() -> extenderMotor.set(extenderPIDController.calculate(getDistance(), height)), this).finallyDo(() -> stop());
        } else {
            stop();
            extenderMotor.disable();
            DataLogManager.log("Extender Encoder Disconnected");
            return new InstantCommand();
        }
    }


    @Override
    public void periodic() {
        totalRotations();
        if (getLimitSwitch()) reset();
        
    }
}