package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private double motorSpeed;
    private Constants.GamePiece piece;

    public Intake(IntakeSubsystem intakeSubsystem, double motorSpeed, Constants.GamePiece piece) {
        this.intakeSubsystem = intakeSubsystem;
        this.motorSpeed = motorSpeed;
        this.piece = piece;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntake(motorSpeed);
    }

    @Override
    public boolean isFinished() {
        int distance = intakeSubsystem.getDistance();
        if (distance > 1230 && piece == GamePiece.CORAL) {
            return true;
        } else if (distance > 600 && piece == GamePiece.ALGAE) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
