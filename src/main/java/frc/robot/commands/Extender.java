package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.ExtenderSubsystem;

public class Extender extends Command {
    private ExtenderSubsystem extenderSubsystem;

    private ExtenderHeights height;

    public enum ExtenderHeights {
        L2,
        L3,
        L4
    }


    public Extender(ExtenderSubsystem extenderSubsystem, ExtenderHeights height) {
        this.extenderSubsystem = extenderSubsystem;
        this.height = height;

        addRequirements(extenderSubsystem);
    }


    @Override
    public void execute() {
        extenderSubsystem.extend(extenderSubsystem.outputScalar(extenderSubsystem.getDistance(), extenderSubsystem.getTargetHeight(height)));
    }

    public boolean isFinished() {
        switch (height) {
            case L2:
                return extenderSubsystem.getDistance() > ExtenderConstants.dL2Height;
            case L3:
                return extenderSubsystem.getDistance() > ExtenderConstants.dL3Height;
            case L4:
                return extenderSubsystem.getDistance() > ExtenderConstants.dL4Height;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}
