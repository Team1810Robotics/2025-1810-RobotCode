/* package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private boolean in;

    public Intake(IntakeSubsystem intakeSubsystem, boolean in) {
        this.intakeSubsystem = intakeSubsystem;
        this.in = in;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (in) {
            intakeSubsystem.intakeIn();
        } else {
            intakeSubsystem.intakeOut();
        }      
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
  */