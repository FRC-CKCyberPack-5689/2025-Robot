// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RMap.ArmPositions;
import frc.robot.RMap.Globals;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Globals.myled.setData(Globals.myledBuff);
    Globals.rainbowfinal.applyTo(Globals.myledBuff);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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

    Globals.drive_SBS.setSlow(false);
    Globals.arm_SBS.setArm(Globals.armMode.HOME).schedule();
    if (Globals.arm_SBS.isLocked()) {
      Globals.arm_SBS.lockArm();
    }

    if (Globals.drive_SBS.drive_type) {
      SmartDashboard.putString("DriveType", "Gyro Drive");
    } else {
      SmartDashboard.putString("DriveType", "Relative Drive");
    }
    
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("current angle",Globals.gyro.getAngle());
  }

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
}
