package frc.robot.subsystems.superstructure;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.RobotConstants.IntakeConstants.IntakeMode;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    private ColorSensorV3 colorSensor;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

        colorSensor = new ColorSensorV3(Port.kOnboard);

        Shuffleboard.getTab("Intake").addNumber("Distance", () -> getDistance());
        Shuffleboard.getTab("Intake").addBoolean("Distance Sensor", () -> colorSensor.isConnected());
        Shuffleboard.getTab("Intake").addNumber("Blue", () -> colorSensor.getBlue());
    }

    public int getDistance() {
        return colorSensor.getProximity();
    }

    public int getBlue() {
        return colorSensor.getBlue();
    }

    /**
     * Checks if a coral has been intaked by using distance and color from the color sensor
     * @return true if Coral is present, false otherwise
     */
    public boolean isCoralPresent() {
        return getDistance() > 2000 && getBlue() > 10;
    }

    /**
     * Checks if an algae has been intaked by using distance and color from the color sensor
     * @return true if Algae is present, false otherwise
     */
    public boolean isAlgaePresent() {
        return getDistance() > 1800 && getBlue() < 10;
    }

    public void stop() {
        intakeMotor.stopMotor();
    }


    /**
     * Runs the intake motor with the specified mode
     * 
     * @param mode The mode to run the intake in
     * @return Command to run the motor
     */
    public Command run(IntakeConstants.IntakeMode mode) {        
        switch (mode) {
            case IN:
                if (isAlgaePresent()) {
                    // Detects if an algae has been intaked and idles it instead
                    return Commands.startEnd(() -> intakeMotor.set(0.15), () -> stop(), this).until(() -> isFinished(mode));
                } else {
                    return Commands.startEnd(() -> intakeMotor.set(1), () -> stop(), this).until(() -> isFinished(mode));
                }
            case OUT:
                return Commands.startEnd(() -> intakeMotor.set(-0.15), () -> stop(), this);
            case KICK:
                return Commands.startEnd(() -> intakeMotor.set(-1), () -> stop(), this).until(() -> isFinished(mode));
            case STOP:
                return Commands.startEnd(() -> intakeMotor.stopMotor(), () -> stop(), this).until(() -> isFinished(mode));
            default:
                return Commands.startEnd(() -> intakeMotor.stopMotor(), () -> stop(), this).until(() -> isFinished(mode));
        }
    }

    /**
     * Checks if the intake should stop based on the mode and sensor readings
     * 
     * @param mode The current mode of the intake
     * @return true if the intake should stop, false otherwise
     */
    public boolean isFinished(IntakeMode mode) {
        //If we are intaking, the piece is a coral, and it is closer than 2000, stop
        if (isCoralPresent() && mode == IntakeMode.IN) {
            return true;
        } 

        if (mode == IntakeMode.STOP) {
            return true;
        }

        return false;
    }
}