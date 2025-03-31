package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtenderSubsystem;

public class Home extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private boolean die = false;

    /**
     * Create a new Extender command.
     *
     * @param extenderSubsystem The {@link ExtenderSubsystem} to control.
     * @param height            The height to extend to.
     */
    public Home(ExtenderSubsystem extenderSubsystem) {
        this.extenderSubsystem = extenderSubsystem;

        addRequirements(extenderSubsystem);
    }

    @Override
    public void execute() {

        if (!extenderSubsystem.getLimitSwitch()){
            extenderSubsystem.run(-.15);
        }

        if (extenderSubsystem.getLimitSwitch()) {
            extenderSubsystem.reset();
            extenderSubsystem.extend(.5);
            die = true;
        }

    }

    public boolean isFinished() {
        return die;
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}