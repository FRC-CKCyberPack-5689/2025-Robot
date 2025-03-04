// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RMap.*;
import frc.robot.RMap.Globals.POV;
import frc.robot.RMap.Globals.armMode;

public class arm_subsystem extends SubsystemBase {
  /** Creates a new arm_subsystem. */
  private SparkMax armBase, armJoint;
  private SparkFlex intake;
  private SparkClosedLoopController armBaseCon, armJointCon;

  private boolean isLocked = false;
  private RelativeEncoder armBaseEnc, armJointEnc;
  // Set to Y/HOME by default.
  // x pressed: coral deposit options
    // def/1, 2, 3, 4 for each coral lvl respective.
  // a pressed: algae pick up options
    // def/3: ground
    // 1: lvl 1
    // 2: lvl 2
  // b pressed: coral pick up options
    // def/1: station pick up
    // 2: ground pick up
  // y pressed: reset to home
    // def/1: home
    // 2: processor
  // start button (left of XBOX) climb prime
  // option button (right of XBOX) climb collapse

  private armMode currentMode;
  private MotorPositions currentPos;

  public arm_subsystem() {
    armBase  = new SparkMax(MotorConstants.kFR_ARM_BASE_ID,  MotorType.kBrushless);
    armJoint      = new SparkMax(MotorConstants.kARM_JOINT_ID,    MotorType.kBrushless);
    intake        = new SparkFlex(MotorConstants.kINTAKE_ID,       MotorType.kBrushless);

    armBaseCon = armBase.getClosedLoopController();
    armJointCon = armJoint.getClosedLoopController();

    armBaseEnc = armBase.getEncoder();
    armJointEnc = armJoint.getEncoder();
    }

  @Override
  public void periodic() {
    if (currentPos == ArmPositions.Home) {
      Globals.drive_SBS.setSlow(false);
    } else {
      Globals.drive_SBS.setSlow(true);
    }
  }

  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }

  public void stopIntake() {
    intake.set(0);
  }

  private void writeMotors(MotorPositions positions) {
    if (positions == ArmPositions.Home) {
      armBaseCon.setReference(armBaseEnc.getPosition()/2, ControlType.kPosition);
      armJointCon.setReference(armJointEnc.getPosition()/2, ControlType.kPosition);
      new WaitCommand(0.5).andThen(() -> {
        armBaseCon.setIAccum(0);
        armJointCon.setIAccum(0);
        armBaseCon.setReference(ArmPositions.Home.k_armBasePosition, ControlType.kPosition);
        armJointCon.setReference(ArmPositions.Home.k_armJointPosition, ControlType.kPosition);
      }).schedule();
    } else {
      armBaseCon.setReference(positions.k_armBasePosition, ControlType.kPosition);
      armJointCon.setReference(positions.k_armJointPosition, ControlType.kPosition);
    }
    currentPos = positions;
  }

  public Command setArm(armMode mode) {
    return this.runOnce(() -> {
      if (!isLocked) {
        currentMode = mode;

        switch(mode) {
          case REEF:
          writeMotors(ArmPositions.CoralLVL1);
          break;

          case ALGAE:
          writeMotors(ArmPositions.BallLVL1);
          break;

          case HOME:
          writeMotors(ArmPositions.Home);
          break;

          case CORAL:
          writeMotors(ArmPositions.CoralSIn);
          break;

          case CLIMB:
          writeMotors(ArmPositions.ClimbPrime);
          break;
        }
      }
    });
  }

  public Command setArm(POV povInput) {
    return this.runOnce(() -> {
      if (!isLocked) {
        switch (currentMode) {
          case REEF:
          switch(povInput) {
            default:
            writeMotors(ArmPositions.CoralLVL1);
            break;

            case UP:
            writeMotors(ArmPositions.CoralLVL1);
            break;

            case RIGHT:
            writeMotors(ArmPositions.CoralLVL2);
            break;

            case DOWN:
            writeMotors(ArmPositions.CoralLVL3);
            break;

            case LEFT:
            writeMotors(ArmPositions.CoralLVL4);
            break;
          }
          break;

          case ALGAE:
          switch(povInput) {
            default:
            writeMotors(ArmPositions.BallLVL1);
            break;

            case UP:
            writeMotors(ArmPositions.BallLVL1);
            break;

            case RIGHT:
            writeMotors(ArmPositions.BallLVL2);
            break;

            case DOWN:
            writeMotors(ArmPositions.BallG);
            break;

            case LEFT:
            writeMotors(ArmPositions.BallTower);
            break;
          }
          break;

          case HOME:
          switch(povInput) {
            default:
            writeMotors(ArmPositions.Home);
            break;

            case UP:
            writeMotors(ArmPositions.Home);
            break;
          }
          break;

          case CORAL:
          switch(povInput) {
            default:
            writeMotors(ArmPositions.CoralSIn);
            break;

            case UP:
            writeMotors(ArmPositions.CoralSIn);
            break;

            case RIGHT:
            writeMotors(ArmPositions.CoralGInPrep);
            break;

            case DOWN:
            writeMotors(ArmPositions.CoralGIn);
            break;
          }
          break;
        
          case CLIMB:
          switch(povInput) {
            default:
            writeMotors(ArmPositions.ClimbPrime);
            break;

            case UP:
            writeMotors(ArmPositions.ClimbPrime);
            break;

            case RIGHT:
            writeMotors(ArmPositions.ClimbCollapse);
            break;
          }
          break;

          default:
          break;
        }
      }
    });
  }


  public void lockArm() {
    // if (currentMode == Globals.armMode.CLIMB) {
      isLocked = !isLocked; 
    if (isLocked) {
      Globals.armServo.setPulseWidth(SpeedConstants.kARM_SERVO_LOCK);
      Globals.baseServo.setPulseWidth(SpeedConstants.kBASE_SERVO_LOCK);
    } else {
      Globals.armServo.setPulseWidth(SpeedConstants.kARM_SERVO_UNLOCK);
      Globals.baseServo.setPulseWidth(SpeedConstants.kBASE_SERVO_UNLOCK);
    }
    // }
  }

  public boolean isLocked() {
    return isLocked;
  }
}
