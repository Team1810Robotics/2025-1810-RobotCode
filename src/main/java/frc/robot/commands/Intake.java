package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private double motorSpeed;

    public Intake(IntakeSubsystem intakeSubsystem, double motorSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.motorSpeed = motorSpeed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntake(motorSpeed);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getIntakeBeam();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
