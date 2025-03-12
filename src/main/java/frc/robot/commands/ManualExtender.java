package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.ExtenderSubsystem;

public class ManualExtender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private double height;

    /**
     * Create a new Extender command.
     *
     * @param extenderSubsystem The {@link ExtenderSubsystem} to control.
     * @param height            Runs up if {@link ExtenderConstants#L2_HEIGHT},
     *                          run down if {@link ExtenderConstants#L4_HEIGHT}
     */
    public ManualExtender(ExtenderSubsystem extenderSubsystem, double height) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;

        addRequirements(extenderSubsystem);
    }


    @Override
    public void execute() {
        if (height == ExtenderConstants.BASE_HEIGHT) {
           extenderSubsystem.run(-.15);
        } else if (height == ExtenderConstants.L2_HEIGHT) {
           extenderSubsystem.run(.2);
        }

    }


    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}