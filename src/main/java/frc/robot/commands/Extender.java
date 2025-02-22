package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ExtenderConstants.ExtenderHeights;
import frc.robot.subsystems.ExtenderSubsystem;

public class Extender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private ExtenderHeights height;




    public Extender(ExtenderSubsystem extenderSubsystem, ExtenderHeights height) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;

        addRequirements(extenderSubsystem);
    }


    @Override
    public void execute() {
        // extenderSubsystem.extend(extenderSubsystem.getTargetHeight(height));
        if (height == ExtenderHeights.L2) {
            extenderSubsystem.run(-.5);
        } else if (height == ExtenderHeights.L4) {
            extenderSubsystem.run(.5);
        }

    }


    public boolean isFinished() {
        // double targetHeight = extenderSubsystem.getTargetHeight(height);
        // double currentHeight = extenderSubsystem.getDistance();

        // if (Math.abs(targetHeight - currentHeight) < 0.1) {
        //     return true;
        // }

        // // If the extender is moving upwards and has reached the target height, or if it is moving
        // // downwards and has passed the target height, then return true
        // if ((targetHeight > currentHeight && extenderSubsystem.getDistance() > currentHeight) ||
        //     (targetHeight < currentHeight && extenderSubsystem.getDistance() < currentHeight)) {
        //     return true;
        // }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}