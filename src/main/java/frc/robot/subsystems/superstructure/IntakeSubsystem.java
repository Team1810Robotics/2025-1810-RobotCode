package frc.robot.subsystems.superstructure;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;

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

    public void stop() {
        intakeMotor.stopMotor();
    }

    public boolean end(IntakeMode mode) {
        int distance = getDistance();
        int blue = getBlue();

        if (distance > 2000 && mode == IntakeMode.IN && blue > 10) {
            return true;
        } 

        if (mode == IntakeMode.STOP) {
            return true;
        }

        return false;
    }
}