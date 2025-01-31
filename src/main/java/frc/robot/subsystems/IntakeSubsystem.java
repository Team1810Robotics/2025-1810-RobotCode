package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;


public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    private DigitalInput intakeBeam;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

        intakeBeam = new DigitalInput(IntakeConstants.INTAKE_BEAM_ID);
    }



    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public boolean getIntakeBeam() {
        return !intakeBeam.get();
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
