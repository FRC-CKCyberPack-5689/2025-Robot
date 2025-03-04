// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap.*;

public class drive_train_subsystem extends SubsystemBase {
  /** Creates a new drive_train_subsystem. */
  private SparkMax front_left;
  private SparkMax front_right;
  private SparkMax back_left;
  private SparkMax back_right;
  private MecanumDrive mecanumDrive;
  public boolean drive_type = false;
  private double lastPress = 0;
  private boolean isSlow;
  //false is relative, true is gyro

  public drive_train_subsystem() {
    front_left    = new SparkMax(MotorConstants.kFL_WHEEL_ID, MotorType.kBrushless);
    front_right   = new SparkMax(MotorConstants.kFR_WHEEL_ID, MotorType.kBrushless);
    back_left     = new SparkMax(MotorConstants.kBL_WHEEL_ID, MotorType.kBrushless);
    back_right    = new SparkMax(MotorConstants.kBR_WHEEL_ID, MotorType.kBrushless);
    
    mecanumDrive = new MecanumDrive(front_left, back_left, front_right, back_right);
    mecanumDrive.setDeadband(0.05);
  }

  @Override
  public void periodic() {}

  public void driveCartesian(double x,double y,double r) {
    mecanumDrive.driveCartesian(-x, y, r );
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }

  public void changeDrive() {
      drive_type = !drive_type; 
      if (drive_type) {
        SmartDashboard.putString("DriveType", "Gyro Drive");
      } else {  
        SmartDashboard.putString("DriveType", "Relative Drive");
      }
  }

  /*TODO: This will happen so fast you wont even see the robot move */
  public Command autoDriveForward() {
    return this.runEnd(() -> driveCartesian(0.05, 0, 0), () -> stop());
  }

  public void resetGyro() {
    if (Timer.getFPGATimestamp() - lastPress < 0.5) {
      Globals.gyro.reset();
    }
    lastPress = Timer.getFPGATimestamp();
  }

  public void setSlow(boolean slow) {
    isSlow = slow;
  }
  public boolean isSlow() {
    return isSlow;
  }
}
