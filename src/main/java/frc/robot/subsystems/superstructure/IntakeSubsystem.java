package frc.robot.subsystems.superstructure;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    public void run(IntakeConstants.IntakeMode mode) {
        int blue = getBlue();
        int distance = getDistance();
        
        
        switch (mode) {
            case IN:
                if (blue < 10 && distance > 1800) {
                    // Detects if an algae has been intaked and idles it instead
                    intakeMotor.set(0.1);
                } else {
                    intakeMotor.set(1);
                }
                break;
            case OUT:
                intakeMotor.set(-0.15);
                break;
            case KICK:
                intakeMotor.set(-1);
                break;
            case STOP:
                intakeMotor.stopMotor();
                break;
            default:
                intakeMotor.stopMotor();
                break;
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