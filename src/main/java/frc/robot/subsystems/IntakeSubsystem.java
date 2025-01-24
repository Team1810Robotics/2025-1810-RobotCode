package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    }



    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
