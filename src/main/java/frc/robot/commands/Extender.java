package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtenderSubsystem;

public class Extender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private double height;
    private double startTime;
    private boolean die = false;

    /**
     * Create a new Extender command.
     *
     * @param extenderSubsystem The {@link ExtenderSubsystem} to control.
     * @param height            The height to extend to.
     */
    public Extender(ExtenderSubsystem extenderSubsystem, double height) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;

        startTime = Timer.getFPGATimestamp();

        addRequirements(extenderSubsystem);
    }

    public Extender(ExtenderSubsystem extenderSubsystem, double height, boolean die) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;
        this.die = die;
    }


    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();

        if (extenderSubsystem.getLimitSwitch()) {
            extenderSubsystem.reset();
            extenderSubsystem.extend(.5);
        } else if (currentTime - startTime > 4) {
            extenderSubsystem.extend(height);
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