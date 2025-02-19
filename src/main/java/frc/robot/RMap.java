// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive_train_subsystem;

/** Add your docs here. */
public class RMap {

    public static class Globals{
        public static drive_train_subsystem drive_SBS;
        
        public static CommandXboxController controller;
    }

    public class MotorConstants{

        //these are the drive motors ethan <<< did nathan write this?
        public static final int kFL_WHEEL_ID = 2;
        public static final int kFR_WHEEL_ID = 3;
        public static final int kBL_WHEEL_ID = 5;
        public static final int kBR_WHEEL_ID = 4;

        public static final int kFR_ARM_BASE_ID = 7;
        //REAR IS FOLLOWING FRONT
        public static final int kRE_ARM_BASE_ID = 6;

        public static final int kARM_JOINT_ID = 7;
        
        public static final int kWRIST_JOINT_ID = 8;

        public static final int kINTAKE_ID = 9;
    }

    public class SpeedConstants{
        public static final double kDRIVE_TRAIN_MAX_SPEED = 1;
        public static final double kDRIVE_TRAIN_MAX_ACCEL = 0.1;

        public static final class kARM_BASE {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0;
        }
        
        public static final double kBALL_INTAKE = 2;
        public static final double kCORAL_INTAKE = -1;
        public static final double kBALL_SHOOT = -2;
        public static final double kCORAL_SHOOT = 1;
    }

    public class IOConstants{
        public static final int kCONTROLLER_PORT = 0;
        
    }

    public class Suppliers{
        
    }
}
