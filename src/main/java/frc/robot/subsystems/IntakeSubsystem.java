package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ShuffleboardTabs;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    private ColorSensorV3 colorSensor;

    private final ShuffleboardTab tab = ShuffleboardTabs.INTAKE;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

        colorSensor = new ColorSensorV3(Port.kOnboard);

        tab.addNumber("Distance", () -> getDistance());
        tab.addBoolean("Distance Sensor", () -> colorSensor.isConnected());
        tab.addNumber("Blue", () -> colorSensor.getBlue());
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