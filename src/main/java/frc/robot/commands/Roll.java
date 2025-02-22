// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollSubsystem;

public class Roll extends Command {

  private RollSubsystem rollSubsystem;
  private double setpoint;


  public Roll(RollSubsystem rollSubsystem, double setpoint) {
    this.rollSubsystem = rollSubsystem;
    this.setpoint = setpoint;

    addRequirements(rollSubsystem);
  }


  @Override
  public void execute() {
    rollSubsystem.run(setpoint);
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
