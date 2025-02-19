// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.Globals;
import frc.robot.RMap.SpeedConstants;

public class tele_drive extends Command {
  /** Creates a new tele_drive. */
  private double fwd_speed, strafe_speed, rot_speed;

  public tele_drive() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(Globals.drive_SBS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fwd_speed = 0;
    strafe_speed = 0;
    rot_speed  = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //UNCOMMENT WHEN YOU GET GYRO WORKING
    // if (Globals.drive_SBS.drive_type) {
    //   fwd_speed = calculateGyroDrive(Globals.controller.getLeftX(),Globals.controller.getLeftY(), fwd_speed, 0);
    //   strafe_speed = calculateGyroDrive(Globals.controller.getLeftX(), Globals.controller.getLeftY(), strafe_speed, 1);
    //   rot_speed = calculateDrive(Globals.controller.getRightX(), rot_speed);
    // } else {
    //   fwd_speed = calculateDrive(Globals.controller.getLeftY(), fwd_speed);
    //   strafe_speed = calculateDrive(Globals.controller.getLeftX(), strafe_speed);
    //   rot_speed = calculateDrive(Globals.controller.getRightX(), rot_speed);
    // }

    fwd_speed = calculateDrive(Globals.controller.getLeftY(), fwd_speed);
    strafe_speed = calculateDrive(Globals.controller.getLeftX(), strafe_speed);
    rot_speed = calculateDrive(Globals.controller.getRightX(), rot_speed);
    Globals.drive_SBS.driveCartesian(fwd_speed, strafe_speed, rot_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * @param input Controller stick input
   * @param cspeed Current axis speed of robot
   * @return Ramped and cliped motor speed 
   */

  public double calculateDrive(double input, double cspeed) {
    double calculated_speed = 0;
    double i = MathUtil.applyDeadband(input, 0.05);
    calculated_speed = (i - cspeed) * SpeedConstants.kDRIVE_TRAIN_MAX_ACCEL + cspeed;

    return calculated_speed;
  }

  // public double calculateGyroDrive(double input_x, double input_y, double cspeed,int dir) {
  //   double calculated_speed = 0;
  //   double ix = MathUtil.applyDeadband(input_x, 0.05);
  //   double iy = MathUtil.applyDeadband(input_y, 0.05);
  //   double angle = Globals.gyro.getAngle();

  //   if (dir == 0) {
  //     calculated_speed=Math.cos(angle*Math.PI/180)*iy - Math.sin(angle*Math.PI/180)*ix;
  //   } else {
  //     calculated_speed=Math.sin(angle*Math.PI/180)*iy*1.3 + Math.cos(angle*Math.PI/180)*ix*1.3;
  //   }
  //   calculated_speed = (calculated_speed - cspeed) * SpeedConstants.kDRIVE_TRAIN_MAX_ACCEL + cspeed;

  //   return calculated_speed;
  // }
}
