package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.Mode;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private IntakeConstants.Mode piece;

    //runs when true
    private boolean toggle = false;

    public Intake(IntakeSubsystem intakeSubsystem, IntakeConstants.Mode piece) {
        this.intakeSubsystem = intakeSubsystem;
        this.piece = piece;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (piece == Mode.ALGAE) {
            toggle = !toggle;
        }
        
    }

    @Override
    public void execute() {
        if (piece == Mode.CORAL) {
            intakeSubsystem.setSpeed(1);
        } else if (piece == Mode.ALGAE){
            intakeSubsystem.setSpeed(.75);
        } else if (piece == Mode.OUT){
            intakeSubsystem.setSpeed(-.5);
        } else {
            intakeSubsystem.setSpeed(.05);
        }
    }

    @Override
    public boolean isFinished() {
        int distance = intakeSubsystem.getDistance();

        if (piece == Mode.ALGAE) {
            return !toggle;
        }

        if (distance > 2000 && piece == Mode.CORAL) {
            return true;
        } 

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}