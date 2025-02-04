package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollSubsystem;

public class Roll extends Command{
    private RollSubsystem rollSubsystem;

    private double setPoint;

    public Roll(RollSubsystem rollSubsystem, double setPoint) {
        this.rollSubsystem = rollSubsystem;
        this.setPoint = setPoint;
        
        addRequirements(rollSubsystem);
    }

    @Override
    public void execute() {
        rollSubsystem.runRoll(setPoint);
    }


    @Override
    public boolean isFinished() {
        return false;
        // return rollSubsystem.atSetPoint(setPoint);
    }
    @Override
    public void end(boolean interrupted) {
        rollSubsystem.stop();
    }
}
