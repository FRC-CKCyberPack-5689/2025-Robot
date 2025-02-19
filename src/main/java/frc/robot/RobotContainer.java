// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap.Globals;
import frc.robot.RMap.IOConstants;
import frc.robot.commands.tele_drive;
import frc.robot.subsystems.drive_train_subsystem;

public class RobotContainer {
  public RobotContainer() {
    // Globals.gyro = new AHRS(IOConstants.kGYRO);
    Globals.drive_SBS = new drive_train_subsystem();


    Globals.controller = new CommandXboxController(IOConstants.kCONTROLLER_PORT);

    // autonomous_command = new SendableChooser<>();
    // autonomous_command.setDefaultOption("None", null);
    // autonomous_command.addOption("drive test", new auto_drive().withTimeout(3));
    // SmartDashboard.putData(autonomous_command);

    configureBindings();
  }

  private void configureBindings() {
    Globals.drive_SBS.setDefaultCommand(new tele_drive());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
