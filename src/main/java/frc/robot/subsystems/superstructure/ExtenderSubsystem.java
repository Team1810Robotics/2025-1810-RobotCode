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
    private final PIDController extenderPIDController;
    private double currentSetpoint;
    private DigitalInput limitSwitch;
    

    public ExtenderSubsystem() {
        extenderMotor = new TalonFX(ExtenderConstants.MOTOR_ID);
        limitSwitch = new DigitalInput(ExtenderConstants.LIMIT_SWITCH_ID);

        extenderMotor.getConfigurator().apply(Configs.getExtenderConfig());

        extenderPIDController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD);
        extenderPIDController.setTolerance(ExtenderConstants.TOLERANCE);

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getDistance());
        Shuffleboard.getTab("Extender").addBoolean("Endstop", () -> getLimitSwitch());
        Shuffleboard.getTab("Extender").add("Extender PID", extenderPIDController);
        Shuffleboard.getTab("Extender").addNumber("Extender PID Out", () -> extenderPIDController.calculate(getDistance(), currentSetpoint));
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

    /**
     * Get the current encoder value in rotations (relative to zero position)
     */
    public double getEncoder() {
        return (extenderMotor.getPosition().getValueAsDouble());
    }

    /**
     * Returns the total distance the extender has extended, in inches.
     */
    public double getDistance() {
        return getEncoder() * ExtenderConstants.INCHES_PER_ROTATION;
    }

    /**
     * Runs the extender motor manually.
     */
    public Command runManual(double speed) {
        return Commands.run(() -> {
            if (getLimitSwitch()) reset();
            extenderMotor.set(speed);
        }, this).finallyDo(() -> extenderMotor.set(0));
    }

    /**
     * Runs the extender motor with PID control.
     */
    public Command run(double height) {
        currentSetpoint = height;
        return Commands.run(() -> {
            double targetRotations = height / ExtenderConstants.INCHES_PER_ROTATION;
            double output = extenderPIDController.calculate(getEncoder(), targetRotations);
            extenderMotor.set(output);
        }, this).finallyDo(() -> stop());
    }

    /**
     * Resets the extender position when limit switch is hit
     */
    public void reset() {
        extenderMotor.setPosition(0);
        run(.25);
    }

    @Override
    public void periodic() {
        if (getLimitSwitch()) {
            reset();
        }
    }
}