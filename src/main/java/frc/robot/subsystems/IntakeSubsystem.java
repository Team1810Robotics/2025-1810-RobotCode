package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.I2C;


public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    private ColorSensorV3 colorSensor;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }



    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }


    public int getColorSensor() {
        return colorSensor.getProximity();
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
