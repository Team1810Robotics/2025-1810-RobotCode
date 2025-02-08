package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    public IntakeSubsystem() {
        //TODO: Figure out this port
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    }

    public void intakeIn() {
        intakeMotor.set(1);
    }

    public void intakeOut() {
        intakeMotor.set(-1);
    }

    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}