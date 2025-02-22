// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends Command {

  ArmSubsystem armSubsystem;
  double setpoint;

  public Arm(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.useOutput(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
