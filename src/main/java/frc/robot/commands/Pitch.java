// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PitchSubsystem;

public class Pitch extends Command {

  private PitchSubsystem pitchSubsystem;
  private double setpoint;
  private boolean die = false;


  /**
   * Pitch command.
   *
   * @param pitchSubsystem Pitch subsystem.
   * @param setpoint       Setpoint in degrees, found in {@link PitchConstants}
   */
  public Pitch(PitchSubsystem pitchSubsystem, double setpoint) {
    this.pitchSubsystem = pitchSubsystem;
    this.setpoint = setpoint;

    addRequirements(pitchSubsystem);
  }

  public Pitch(PitchSubsystem pitchSubsystem, boolean die) {
    this.pitchSubsystem = pitchSubsystem;
    this.die = die;

    addRequirements(pitchSubsystem);
  }


  @Override
  public void execute() {
    pitchSubsystem.run(setpoint);
  }

  @Override
  public boolean isFinished() {
    return die;
  }

  @Override
  public void end(boolean interrupted) {
    pitchSubsystem.stop();
  }


}
