// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollSubsystem;

public class Roll extends Command {

  private RollSubsystem rollSubsystem;
  public double setpoint;
  private boolean die = false;

  /**
   * Create a new command that runs the roll subsystem to the specified setpoint.
   *
   * @param rollSubsystem the subsystem to control
   * @param setpoint Setpoint in degreees, setpoints found in {@link RollConstants}
   */
  public Roll(RollSubsystem rollSubsystem, double setpoint) {
    this.rollSubsystem = rollSubsystem;
    this.setpoint = setpoint;

    addRequirements(rollSubsystem);
  }

  public Roll(RollSubsystem rollSubsystem, boolean die) {
    this.rollSubsystem = rollSubsystem;
    this.die = die;

    addRequirements(rollSubsystem);
  }

  @Override
  public void execute() {
    rollSubsystem.run(setpoint);
    // rollSubsystem.runManual(setpoint);
  }

  @Override
  public boolean isFinished() {
    return die;
  }
  
  @Override
  public void end(boolean interrupted) {
    rollSubsystem.stop();
  }

}
