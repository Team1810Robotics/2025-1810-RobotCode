package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtenderSubsystem;

public class Extender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private double height;
    private double startTime;

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


    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();

        if (currentTime - startTime > 4) {
            extenderSubsystem.extend(height);
        }
        
/*         if (height == ExtenderConstants.BASE_HEIGHT) {
           extenderSubsystem.run(-.15);
        } else if (height == ExtenderConstants.L2_HEIGHT) {
           extenderSubsystem.run(.2);
        } */

    }


    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}