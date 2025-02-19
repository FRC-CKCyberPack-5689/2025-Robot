// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap.MotorConstants;

public class arm_subsystem extends SubsystemBase {
  /** Creates a new arm_subsystem. */
  private SparkMax armBaseFront;
  private SparkMax armBaseRear;
  private SparkMax armJoint;
  private SparkMax wristJoint;
  private SparkMax intake;
  private AbsoluteEncoder armBaseEnc;

  public enum ArmPosition {
    LVL1,
    LVL2,
    LVL3,
    LVL4,
    CLIMBPRIME,
    CLIMBCOLLAPSE
  }

  private ArmPosition currentPos;

  public arm_subsystem() {
    armBaseFront = new SparkMax(MotorConstants.kFR_ARM_BASE_ID, MotorType.kBrushless);
    armBaseRear = new SparkMax(MotorConstants.kRE_ARM_BASE_ID, MotorType.kBrushless);
    armJoint = new SparkMax(MotorConstants.kARM_JOINT_ID, MotorType.kBrushless);
    wristJoint = new SparkMax(MotorConstants.kWRIST_JOINT_ID, MotorType.kBrushless);
    intake = new SparkMax(MotorConstants.kINTAKE_ID, MotorType.kBrushless);

    armBaseEnc = armBaseFront.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }

  public void setArmPos(ArmPosition newPos) {
    switch(newPos) {
      case LVL1:
        break;

      case LVL2:
        break;

      case LVL3:
        break;

      case LVL4:
        break;

      case CLIMBPRIME:
        break;

      case CLIMBCOLLAPSE:
        break;
    }
  }

  public void incrArmPos(boolean Up) {
    switch(currentPos) {
      case LVL1:
        if (Up) {
          currentPos = ArmPosition.LVL2;
        }
        break;

      case LVL2:
        if (Up) {
          currentPos = ArmPosition.LVL3;
        } else {
          currentPos = ArmPosition.LVL1;
        }
        break;

      case LVL3:
        if (Up) {
          currentPos = ArmPosition.LVL4;
        } else {
          currentPos = ArmPosition.LVL2;
        }
        break;

      case LVL4:
        if (!Up) {
          currentPos = ArmPosition.LVL3;
        }
        break;

      case CLIMBPRIME:
        currentPos = ArmPosition.CLIMBCOLLAPSE;
        break;

      case CLIMBCOLLAPSE:
        currentPos = ArmPosition.CLIMBPRIME;
        break;

      default:
        currentPos = ArmPosition.LVL1;
    }

    setArmPos(currentPos);
  }

  public void getArmPos() {

  }
}
