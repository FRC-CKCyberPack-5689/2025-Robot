// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap;
import frc.robot.RMap.*;
import frc.robot.RMap.Globals.speedSettings;

public class drive_train_subsystem extends SubsystemBase {
  /** Creates a new drive_train_subsystem. */
  private SparkMax front_left;
  private SparkMax front_right;
  private SparkMax back_left;
  private SparkMax back_right;
  private RelativeEncoder fL_encoder, fR_encoder, bL_encoder, bR_encoder;
  private MecanumDrive mecanumDrive;
  public boolean drive_type = false;
  private double lastPress = 0;
  private PIDController pidCon;
  private MecanumDriveOdometry odometry;

  private speedSettings speed;
  //false is relative, true is gyro

  public drive_train_subsystem() {
    front_left    = new SparkMax(MotorConstants.kFL_WHEEL_ID, MotorType.kBrushless);
    front_right   = new SparkMax(MotorConstants.kFR_WHEEL_ID, MotorType.kBrushless);
    back_left     = new SparkMax(MotorConstants.kBL_WHEEL_ID, MotorType.kBrushless);
    back_right    = new SparkMax(MotorConstants.kBR_WHEEL_ID, MotorType.kBrushless);

    fL_encoder = front_left.getEncoder();
    fR_encoder = front_right.getEncoder();
    bL_encoder = back_left.getEncoder();
    bR_encoder = back_right.getEncoder();

    pidCon = new PIDController(0.0115, 0, 0.0001);
    
    mecanumDrive = new MecanumDrive(front_left, back_left, front_right, back_right);
    mecanumDrive.setDeadband(0.05);

    odometry = new MecanumDriveOdometry(
      SpeedConstants.kDriveKinematics,
      new Rotation2d(getHeading()),
      new MecanumDriveWheelPositions());
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

  
  public void setMotorControllerVolts(
    double frontLeftVoltage,
    double frontRightVoltage,
    double rearLeftVoltage,
    double rearRightVoltage) {
    front_left.setVoltage(frontLeftVoltage);
    front_right.setVoltage(frontRightVoltage);
    back_left.setVoltage(rearLeftVoltage);
    back_right.setVoltage(rearRightVoltage);
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      fL_encoder.getVelocity(),
      fR_encoder.getVelocity(),
      bL_encoder.getVelocity(),
      bR_encoder.getVelocity());
  }

  public MecanumDriveWheelPositions getWheelPosition() {
    return new MecanumDriveWheelPositions(
      fL_encoder.getPosition(),
      fR_encoder.getPosition(),
      bL_encoder.getPosition(),
      bR_encoder.getPosition());
  }

  public void zeroHeading() {
    Globals.gyro.reset();
  }

  public double getHeading() {
    return -Globals.gyro.getAngle();
  }

  public double getTurnRate() {
    return -Globals.gyro.getRate();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getWheelPosition(), pose);
  }

  public void setSafety(boolean x) {
    mecanumDrive.setSafetyEnabled(x);
  }

  

  public MecanumControllerCommand generateTrajectoryCMD(Trajectory trajectory) {
        MecanumControllerCommand controller =
            new MecanumControllerCommand(
                trajectory,
                Globals.drive_SBS::getPose,
                SpeedConstants.kFeedForward,
                SpeedConstants.kDriveKinematics,
                new PIDController(SpeedConstants.kPXController, 0, 0),
                new PIDController(SpeedConstants.kPYController, 0, 0),
                new ProfiledPIDController(SpeedConstants.kPThetaController, 0, 0, SpeedConstants.kThetaControllerConstraints),
                SpeedConstants.kMaxMetersPerSecond,
                new PIDController(0.5, 0, 0),
                new PIDController(0.5, 0, 0),
                new PIDController(0.5, 0, 0),
                new PIDController(0.5, 0, 0),
                Globals.drive_SBS::getWheelSpeeds,
                Globals.drive_SBS::setMotorControllerVolts,
                Globals.drive_SBS);
        
        return controller;
    }
}