package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.Mode;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private IntakeConstants.Mode mode;


    /**
     * Intake command that runs the intake motor at a certain speed depending on
     * the {@link IntakeConstants.Mode} given.
     *
     * @param intakeSubsystem The IntakeSubsystem to run the command on.
     * @param mode           The mode to run the command in. If it's {@link IntakeConstants.Mode#ALGAE}
     *                        the command will toggle the intake on and off.
     */
    public Intake(IntakeSubsystem intakeSubsystem, IntakeConstants.Mode mode) {
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (mode == Mode.ALGAE) {
        }
        
    }

    @Override
    public void execute() {
        if (mode == Mode.CORAL) {
            intakeSubsystem.run(1);
        } else if (mode == Mode.ALGAE){
            intakeSubsystem.run(.75);
        } else if (mode == Mode.IDLE){
            if (intakeSubsystem.getDistance() > 1500){
            intakeSubsystem.run(.1);}
        } else if (mode == Mode.OUT){
            intakeSubsystem.run(-.35);
        } else {
            intakeSubsystem.run(.05);
        }
    }

    @Override
    public boolean isFinished() {
        int distance = intakeSubsystem.getDistance();

        if (distance > 2000 && mode == Mode.CORAL) {
            return true;
        } 

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}