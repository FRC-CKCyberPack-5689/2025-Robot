// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm_subsystem;
import frc.robot.subsystems.drive_train_subsystem;

/** Add your docs here. */
public class RMap {

    public static class Globals{
        public static drive_train_subsystem drive_SBS;
        public static arm_subsystem arm_SBS;

        public static ADIS16470_IMU gyro;
        public static CommandXboxController controller;
        public static PhotonCamera camera;

        public static SendableChooser<Command> autonomous_command;

        public static ServoHub servoH;
        public static ServoChannel baseServo;
        public static ServoChannel armServo;

        // public static LEDPattern rainbow;
        // public static LEDPattern rainbowfinal;

        // public static AddressableLED myled;
        // public static AddressableLEDBuffer myledBuff;

        public static enum armMode {
            REEF,
            HOME,
            ALGAE,
            CORAL,
            CLIMB
          };
        
          public static enum POV {
            UP,
            DOWN,
            LEFT,
            RIGHT,
            AUTO1
          }

          public static enum speedSettings {
            MAX,
            MEDIUM,
            SLOW
          }
    }

    public class MotorConstants{
        /*
         * CAN Bus IDs
         * 0: RoboRIO
         * 1: Power Distribution
         * 2: Front Left Drive Wheel
         * 3: Rear Left Drive Wheel
         * 4: Front Right Drive Wheel
         * 5: Rear Right Drive Wheel
         * 6: Rear-Most Base Arm Joint
         * 7: Front-Most Base Arm Joint
         * 8: Upper Arm Joint
         * 9: Intake Motor
        */

        //these are the drive motors ethan <<< did nathan write this?
        public static final int kFL_WHEEL_ID = 2;
        public static final int kBL_WHEEL_ID = 3;
        public static final int kFR_WHEEL_ID = 4;
        public static final int kBR_WHEEL_ID = 5;
        
        //REAR IS FOLLOWING FRONT
        public static final int kRE_ARM_BASE_ID = 6;
        public static final int kFR_ARM_BASE_ID = 7;
        
        public static final int kARM_JOINT_ID = 8;
        public static final int kINTAKE_ID = 9;

        public static final int kSERVO_HUB_ID = 10;

        public static final ChannelId kARM_SERVO_ID = ChannelId.kChannelId1;
        public static final ChannelId kBASE_SERVO_ID = ChannelId.kChannelId0;
    }

    public class SpeedConstants{
        public static final double kDRIVE_TRAIN_MAX_SPEED = 1;
        public static final double kDRIVE_TRAIN_MAX_ACCEL = 0.1;
        public static final double kDRIVE_TRAIN_MAX_DECEL = 0.2;
        public static final double kDRIVE_TRAIN_ROT_ACCEL = 0.15;
        public static final double kDRIVE_TRAIN_ROT_DECEL = 0.15;

        public static final class kARM_BASE {
            public static final double kP = 0.05999999865889549;
            public static final double kI = 0.00005;
            public static final double kD = 0.12099999934434891;
            public static final double kFF = 0;
            public static final double kMIN = -0.35;
            public static final double kMAX = 0.35;
        }

        public static final class kARM_JOINT {
            public static final double kP = 0.04500000178813934;
            public static final double kI = 0.00001;
            public static final double kD = 0;
            public static final double kFF = 0;
            public static final double kMIN = -0.325;
            public static final double kMAX = 0.325;
        }

        
        public static final double kBALL_INTAKE = -0.4;
        public static final double kBALL_SHOOT = 0.3;
        public static final double kCORAL_INTAKE = 0.7;
        public static final double kCORAL_SHOOT = -0.8;

        public static final int kARM_SERVO_UNLOCK = 1254;
        public static final int kARM_SERVO_LOCK = 1860;

        public static final int kBASE_SERVO_UNLOCK = 1500;
        public static final int kBASE_SERVO_LOCK = 2200;
    }

    public static class MotorPositions {
        public final double k_armBasePosition;
        public final double k_armJointPosition;

        public MotorPositions(double armBase, double armJoint) {
            this.k_armBasePosition = armBase;
            this.k_armJointPosition = armJoint;
        }
    }

    public static class ArmPositions {
        public static MotorPositions CoralLVL1        = new MotorPositions(2.42, 7);
        public static MotorPositions CoralLVL2        = new MotorPositions(4.8, 11);
        public static MotorPositions CoralLVL3        = new MotorPositions(7, 20);
        public static MotorPositions CoralLVL4        = new MotorPositions(7,22);
        public static MotorPositions CoralGInPrep     = new MotorPositions(3,4);
        public static MotorPositions CoralGIn         = new MotorPositions(7,0);
        public static MotorPositions CoralSIn         = new MotorPositions(4.7,14);
        public static MotorPositions BallLVL2         = new MotorPositions(5.4, 15);
        public static MotorPositions BallLVLBump      = new MotorPositions(6.3, 18);
        public static MotorPositions BallLVL1         = new MotorPositions(3.5, 9);
        public static MotorPositions BallG            = new MotorPositions(5.5, 1);
        public static MotorPositions BallInAuto       = new MotorPositions(3, 1);
        public static MotorPositions Home             = new MotorPositions(1, 1);
        public static MotorPositions ClimbPrime       = new MotorPositions(3, 5.5);
        public static MotorPositions ClimbCollapse    = new MotorPositions(1, 2);
    }

    public class IOConstants{
        public static final int kCONTROLLER_PORT = 0;
    }
}
