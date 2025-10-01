package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

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

    public void run(double speed) {
        intakeMotor.set(speed);
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
}