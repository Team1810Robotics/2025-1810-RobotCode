// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollSubsystem;

public class RollCommand extends Command {

  RollSubsystem rollSubsystem;
  double setpoint;

  public RollCommand(RollSubsystem rollSubsystem, double setpoint) {
    this.rollSubsystem = rollSubsystem;
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    rollSubsystem.runRoll(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    rollSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
