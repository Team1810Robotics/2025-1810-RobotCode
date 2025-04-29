package frc.robot.subsystems.superstructure;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.constants.RobotConstants.IntakeConstants;
import frc.robot.util.constants.RobotConstants.IntakeConstants.IntakeMode;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    private ColorSensorV3 colorSensor;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

        colorSensor = new ColorSensorV3(Port.kOnboard);

        intakeMotor.configure(Config.getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Shuffleboard.getTab("Intake").addNumber("Distance", () -> getDistance());
        Shuffleboard.getTab("Intake").addBoolean("Distance Sensor", () -> colorSensor.isConnected());
        Shuffleboard.getTab("Intake").addNumber("Blue", () -> colorSensor.getBlue());
    }

    /**
     * Runs the intake motor with the specified mode
     * 
     * @param mode The mode to run the intake in
     * @return Command to run the motor
     */
    public Command run(IntakeConstants.IntakeMode mode) {
        int blue = getBlue();
        int distance = getDistance();
        
        
        switch (mode) {
            case IN:
                if (blue < 10 && distance > 1800) {
                    // Detects if an algae has been intaked and idles it instead
                    return Commands.startEnd(() -> intakeMotor.set(0.15), () -> stop(), this).until(() -> end(mode));
                } else {
                    return Commands.startEnd(() -> intakeMotor.set(1), () -> stop(), this).until(() -> end(mode));
                }
            case OUT:
                return Commands.startEnd(() -> intakeMotor.set(-0.15), () -> stop(), this);
            case KICK:
                return Commands.startEnd(() -> intakeMotor.set(-1), () -> stop(), this).until(() -> end(mode));
            case STOP:
                return Commands.startEnd(() -> intakeMotor.stopMotor(), () -> stop(), this).until(() -> end(mode));
            default:
                return Commands.startEnd(() -> intakeMotor.stopMotor(), () -> stop(), this).until(() -> end(mode));
        }
    }

    public int getDistance() {
        return colorSensor.getProximity();
    }

    public int getBlue() {
        return colorSensor.getBlue();
    }

    public boolean isCoralPresent() {
        return getDistance() > 2000 && getBlue() > 10;
    }

    public boolean isAlgaePresent() {
        return getDistance() > 2000 && getBlue() < 10;
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Checks if the intake should stop based on the mode and sensor readings
     * 
     * @param mode The current mode of the intake
     * @return true if the intake should stop, false otherwise
     */
    public boolean end(IntakeMode mode) {
        int distance = getDistance();
        int blue = getBlue();

        //If we are intaking, the piece is a coral, and it is closer than 2000, stop
        if (distance > 2000 && mode == IntakeMode.IN && blue > 10) {
            return true;
        } 

        if (mode == IntakeMode.STOP) {
            return true;
        }

        return false;
    }
}