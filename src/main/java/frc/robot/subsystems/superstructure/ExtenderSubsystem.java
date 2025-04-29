package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.constants.RobotConstants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

    private TalonFX extenderMotor;

    private PIDController extenderPIDController;

    private double cumulativeRotations = 0;
    private double previousRotation = 0;

    public double currentSetpoint;

    public DigitalInput limitSwitch;

    public ExtenderSubsystem() {
        extenderMotor = new TalonFX(ExtenderConstants.MOTOR_ID);

        limitSwitch = new DigitalInput(ExtenderConstants.LIMIT_SWITCH_ID);

        extenderMotor.getConfigurator().apply(Config.getExtenderConfig());

        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD);

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getDistance());

        Shuffleboard.getTab("Extender").addNumber("Cumulative Rotations", () -> cumulativeRotations);

        Shuffleboard.getTab("Extender").add("Extender PID", extenderPIDController);

        Shuffleboard.getTab("Extender").addBoolean("Endstop", () -> !limitSwitch.get());
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
        double currentRotation = extenderMotor.getPosition().getValueAsDouble() / 16384;

        // Calculate the delta, accounting for wraparound
        double dRotation = currentRotation - previousRotation;

        // Adjust for wraparound cases
        if (dRotation > 0.5) {
            dRotation -= 1.0; // Wrapped from 1.0 to 0.0 (moving backward)
        } else if (dRotation < -0.5) {
            dRotation += 1.0; // Wrapped from 0.0 to 1.0 (moving forward)
        }

        // Update cumulative position without relying on fullRotations counter
        cumulativeRotations -= dRotation; // Keep negative sign if needed for direction

        // Store for next calculation
        previousRotation = currentRotation;
    }

    public double getEncoder() {
        return extenderMotor.getPosition().getValueAsDouble() / 16384;
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
        return Commands.run(() -> extenderMotor.set(speed), this);
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

        return Commands.run(() -> extenderMotor.set(extenderPIDController.calculate(getDistance(), height)), this).finallyDo(() -> stop());
    }

    public double getExtensionPercent() {
        double percent = (getDistance() - ExtenderConstants.MIN_EXTENSION) / (ExtenderConstants.MAX_EXTENSION - ExtenderConstants.MIN_EXTENSION);
        return Math.max(0, Math.min(percent, 1));
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void reset() {
        cumulativeRotations = 0;
        previousRotation = 0;
        extenderMotor.setPosition(0);
    }

    public void stop() {
        extenderMotor.stopMotor();
    }

    @Override
    public void periodic() {
        totalRotations();
        if (getLimitSwitch()) reset();
        
    }
}