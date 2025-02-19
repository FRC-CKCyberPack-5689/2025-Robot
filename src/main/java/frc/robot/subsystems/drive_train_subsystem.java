// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.RMap.MotorConstants;

public class drive_train_subsystem extends SubsystemBase {
  /** Creates a new drive_train_subsystem. */
  private SparkMax front_left;
  private SparkMax front_right;
  private SparkMax back_left;
  private SparkMax back_right;
  private MecanumDrive mecanumDrive;
  public boolean drive_type = false;
  //false is relative, true is gyro

  public drive_train_subsystem() {
    front_left = new SparkMax(MotorConstants.kFL_WHEEL_ID, MotorType.kBrushless);
    front_right = new SparkMax(MotorConstants.kFR_WHEEL_ID, MotorType.kBrushless);
    back_left = new SparkMax(MotorConstants.kBL_WHEEL_ID, MotorType.kBrushless);
    back_right = new SparkMax(MotorConstants.kBR_WHEEL_ID, MotorType.kBrushless);
    
    mecanumDrive = new MecanumDrive(front_left, back_left, front_right, back_right);
    mecanumDrive.setDeadband(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveCartesian(double x,double y,double r) {
    mecanumDrive.driveCartesian(-x, y, r );
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }
}

// 6.786
// -3.429
// spark max 8