package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtenderSubsystem;

public class Extender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private double height;

    /**
     * Create a new Extender command.
     *
     * @param extenderSubsystem The {@link ExtenderSubsystem} to control.
     * @param height            The height to extend to.
     */
    public Extender(ExtenderSubsystem extenderSubsystem, double height) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;

        addRequirements(extenderSubsystem);
    }


    @Override
    public void execute() {
        extenderSubsystem.extend(height);
        // if (height == ExtenderHeights.BASE) {
        //     extenderSubsystem.run(-.25);
        // } else if (height == ExtenderHeights.L2) {
        //     extenderSubsystem.run(.8);
        // }

    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}