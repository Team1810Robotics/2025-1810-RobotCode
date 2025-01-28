package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PitchSubsystem;

public class Pitch extends Command {
    private PitchSubsystem  pitchSubsystem;
    private double setpoint;

    public Pitch(PitchSubsystem pitchSubsystem, double setpoint) {
        this.pitchSubsystem = pitchSubsystem;
        this.setpoint = setpoint;
        
        addRequirements(pitchSubsystem);
    }

    @Override
    public void execute() {
        pitchSubsystem.runPitch(setpoint);
    }

    @Override
    public boolean isFinished() {
        return pitchSubsystem.atSetPoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        pitchSubsystem.stop();
    }
}
