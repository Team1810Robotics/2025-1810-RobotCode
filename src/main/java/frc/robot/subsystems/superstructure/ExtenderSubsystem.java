package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configs;
import frc.robot.util.constants.RobotConstants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

    private TalonFX extenderMotor;
    public final PIDController extenderPIDController;
    private double currentSetpoint;
    private DigitalInput limitSwitch;

    private double previousRotation = 0;
    private double cumulativeRotations = 0;
    
    public ExtenderSubsystem() {
        extenderMotor = new TalonFX(ExtenderConstants.MOTOR_ID);
        limitSwitch = new DigitalInput(ExtenderConstants.LIMIT_SWITCH_ID);

        extenderMotor.getConfigurator().apply(Configs.getExtenderConfig());

        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD);
        extenderPIDController.setTolerance(ExtenderConstants.TOLERANCE);

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getHeight());
        Shuffleboard.getTab("Extender").addBoolean("Endstop", () -> getLimitSwitch());
        Shuffleboard.getTab("Extender").add("Extender PID", extenderPIDController);
        Shuffleboard.getTab("Extender").addNumber("Extender PID Out", () -> extenderPIDController.calculate(getHeight(), currentSetpoint));
    }

    public boolean atSetpoint() {
        return extenderPIDController.atSetpoint();
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void stop() {
        extenderMotor.stopMotor();
    }

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
     * Get the current encoder value in rotations (relative to zero position)
     */
    public double getEncoder() {
        return extenderMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns the total distance the extender has extended, in inches.
     */
    public double getHeight() {
        return -cumulativeRotations * ExtenderConstants.INCHES_PER_ROTATION;
    }

    /**
     * Runs the extender motor manually at the specified speed.
     * Used by ManualExtender command.
     */
    public void runManual(double speed) {
        extenderMotor.set(speed);
    }

    /**
     * Runs the extender motor with PID control to reach the target height.
     */
    public Command run(double height) {
        currentSetpoint = height;
        return Commands.run(() -> {
            extenderMotor.set(extenderPIDController.calculate(getHeight(), height));
        }, this).finallyDo(() -> stop());
    }

    /**
     * Resets the extender encoder position to zero.
     * Called when limit switch is hit.
     */
    public void reset() {
        extenderMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        totalRotations();
    }
}