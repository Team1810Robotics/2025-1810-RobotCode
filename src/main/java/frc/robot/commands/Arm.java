// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends Command {

  private ArmSubsystem armSubsystem;
  private double setpoint;

  /**
   * Create a new Arm command.
   *
   * @param armSubsystem the {@link ArmSubsystem} to control
   * @param setpoint     the desired angle in degrees
   */
  public Arm(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;

    addRequirements(armSubsystem);
  }

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
