// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final RobotContainer m_robotContainer;

  public AutoFactory autoFactory;

  public boolean encoderAllGood = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
    
    CameraServer.startAutomaticCapture();

    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());

    Shuffleboard.getTab("Teleoperated").add(CommandScheduler.getInstance());

    Shuffleboard.getTab("Teleoperated").addBoolean("Encoder Panic", () -> encoderAllGood);
  }

  @Override
  public void robotInit(){
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @SuppressWarnings("static-access")
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //RobotContainer.ledSubsystem.periodic();

    if (!m_robotContainer.armSubsystem.isEncoderConnected() || !m_robotContainer.extenderSubsystem.isEncoderConnected() || !m_robotContainer.pitchSubsystem.isEncoderConnected() || !m_robotContainer.rollSubsystem.isEncoderConnected()){
      encoderAllGood = false;
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {m_robotContainer.drivetrain.applyRequest(() -> m_robotContainer.brake);}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.configureAutonomus();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.basePosition();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drivetrain.seedFieldCentric();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
