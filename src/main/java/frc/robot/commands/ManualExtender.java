package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.ExtenderSubsystem;
import frc.robot.util.constants.RobotConstants.ExtenderConstants;

public class ManualExtender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private double height;

    /**
     * Create a new Extender command.
     *
     * @param extenderSubsystem The {@link ExtenderSubsystem} to control.
     * @param height            Runs up if {@link ExtenderConstants#L4_HEIGHT},
     *                          run down if {@link ExtenderConstants#BASE_HEIGHT}
     */
    public ManualExtender(ExtenderSubsystem extenderSubsystem, double height) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;

        addRequirements(extenderSubsystem);
    }


    @Override
    public void execute() {

        if (!extenderSubsystem.getLimitSwitch() && height == ExtenderConstants.BASE_HEIGHT) {
           extenderSubsystem.runManual(-.15);
        } else if (!extenderSubsystem.getLimitSwitch() && height == ExtenderConstants.L4_HEIGHT) {
           extenderSubsystem.runManual(.2);
        }

        if (extenderSubsystem.getLimitSwitch()) {
            extenderSubsystem.reset();
            extenderSubsystem.runManual(extenderSubsystem.extenderPIDController.calculate(extenderSubsystem.getHeight(), .5));
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