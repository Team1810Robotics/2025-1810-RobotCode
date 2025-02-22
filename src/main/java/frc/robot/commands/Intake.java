package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private Constants.GamePiece piece;

    public Intake(IntakeSubsystem intakeSubsystem, Constants.GamePiece piece) {
        this.intakeSubsystem = intakeSubsystem;
        this.piece = piece;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (piece == GamePiece.CORAL) {
            intakeSubsystem.setSpeed(1);
        } else if (piece == GamePiece.ALGAE){
            intakeSubsystem.setSpeed(.75);
        } else {
            intakeSubsystem.setSpeed(-.5);
        } 
    }

    @Override
    public boolean isFinished() {
        int distance = intakeSubsystem.getDistance();

        if (distance > 2000 && piece == GamePiece.CORAL) {
            return true;
        } 

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}