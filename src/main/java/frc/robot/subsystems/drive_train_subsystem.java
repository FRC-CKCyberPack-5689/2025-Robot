// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap.*;
import frc.robot.RMap.Globals.speedSettings;

public class drive_train_subsystem extends SubsystemBase {
  /** Creates a new drive_train_subsystem. */
  private SparkMax front_left;
  private SparkMax front_right;
  private SparkMax back_left;
  private SparkMax back_right;
  private MecanumDrive mecanumDrive;
  public boolean drive_type = false;
  private double lastPress = 0;
  private PIDController pidCon;

  private speedSettings speed;
  //false is relative, true is gyro

  public drive_train_subsystem() {
    front_left    = new SparkMax(MotorConstants.kFL_WHEEL_ID, MotorType.kBrushless);
    front_right   = new SparkMax(MotorConstants.kFR_WHEEL_ID, MotorType.kBrushless);
    back_left     = new SparkMax(MotorConstants.kBL_WHEEL_ID, MotorType.kBrushless);
    back_right    = new SparkMax(MotorConstants.kBR_WHEEL_ID, MotorType.kBrushless);

    pidCon = new PIDController(0.0115, 0, 0.0001);
    
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

  public Command autoDriveX(double seconds,double x) {
    return this.run(() -> {
      driveCartesian(x, 0, 0);
    }).withTimeout(seconds);
  }


  public Command autoRotate(double targetAngle) {
    return this.run(() -> {
      double error = pidCon.calculate(targetAngle, -Globals.gyro.getAngle());

      error = MathUtil.clamp(error/2, -0.4, 0.4);
      System.out.println(error);
      driveCartesian(0, 0, -error);
    }).until(() -> -Globals.gyro.getAngle() >= targetAngle-1 && -Globals.gyro.getAngle() <= targetAngle+1).withTimeout(5.0);
  }

  public void resetGyro() {
    if (Timer.getFPGATimestamp() - lastPress < 0.5) {
      Globals.gyro.reset();
    }
    lastPress = Timer.getFPGATimestamp();
  }

  public void setSlow(speedSettings slow) {
    speed = slow;
  }
  public speedSettings isSlow() {
    return speed;
  }
}
