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
     * @param mode           The mode to run the command in        
     */
    public Intake(IntakeSubsystem intakeSubsystem, IntakeConstants.Mode mode) {
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        int blue = intakeSubsystem.getBlue();
        int distance = intakeSubsystem.getDistance();
        
        
        if (mode == Mode.IN && blue < 10 && distance > 1800) {
            //Detects if an algae has been intaked and idles it instead
            intakeSubsystem.run(.1);
        } else if (mode == Mode.IN) {
            intakeSubsystem.run(1);
        } else if (mode == Mode.OUT) {
            intakeSubsystem.run(-.25);
        }
    }

    @Override
    public boolean isFinished() {
        int distance = intakeSubsystem.getDistance();
        int blue = intakeSubsystem.getBlue();

        if (distance > 2000 && mode == Mode.IN && blue > 10) {
            return true;
        }

        if (distance < 1000 && mode == Mode.OUT){
            return true;
        }

        if (mode == Mode.STOP) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}