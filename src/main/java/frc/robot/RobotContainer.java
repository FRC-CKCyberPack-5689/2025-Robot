// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.ServoHub.ResetMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap.Globals;
import frc.robot.RMap.IOConstants;
import frc.robot.RMap.MotorConstants;
import frc.robot.RMap.SpeedConstants;
import frc.robot.commands.tele_drive;
import frc.robot.subsystems.arm_subsystem;
import frc.robot.subsystems.drive_train_subsystem;

public class RobotContainer {
  public RobotContainer() {
    Globals.drive_SBS = new drive_train_subsystem();
    Globals.arm_SBS = new arm_subsystem();
    Globals.gyro = new ADIS16470_IMU();
    Globals.servoH = new ServoHub(MotorConstants.kSERVO_HUB_ID);

    ServoHubConfig config = new ServoHubConfig();

    config.channel0.disableBehavior(BehaviorWhenDisabled.kSupplyPower);
    config.channel1.disableBehavior(BehaviorWhenDisabled.kSupplyPower);

    Globals.servoH.configure(config, ResetMode.kResetSafeParameters);

    Globals.baseServo = Globals.servoH.getServoChannel(MotorConstants.kBASE_SERVO_ID);
    Globals.armServo = Globals.servoH.getServoChannel(MotorConstants.kARM_SERVO_ID);
    Globals.baseServo.setPowered(true);
    Globals.armServo.setPowered(true);
    Globals.baseServo.setEnabled(true);
    Globals.armServo.setEnabled(true);

    Globals.controller = new CommandXboxController(IOConstants.kCONTROLLER_PORT);

    Globals.myled = new AddressableLED(0);
    Globals.myledBuff = new AddressableLEDBuffer(100);

    Globals.rainbow = LEDPattern.rainbow(255, 128);
    Distance distance = Meters.of(1/120.0);
    Globals.rainbowfinal = Globals.rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), distance);
    Globals.rainbowfinal.applyTo(Globals.myledBuff);

    Globals.myled.setLength(Globals.myledBuff.getLength());

    Globals.myled.start();
    

    Globals.autonomous_command = new SendableChooser<>();
    Globals.autonomous_command.setDefaultOption("None", null);
    Globals.autonomous_command.addOption("drive forward", Globals.drive_SBS.autoDriveForward());
    SmartDashboard.putData(Globals.autonomous_command);

    configureBindings();
  }

  private void configureBindings() {
    Globals.drive_SBS.setDefaultCommand(new tele_drive());

    Globals.controller.leftBumper().whileTrue(new StartEndCommand(() -> Globals.arm_SBS.setIntakeSpeed(SpeedConstants.kCORAL_INTAKE),
    () -> Globals.arm_SBS.stopIntake()));
    Globals.controller.rightBumper().whileTrue(new StartEndCommand(() -> Globals.arm_SBS.setIntakeSpeed(SpeedConstants.kBALL_INTAKE),
    () -> Globals.arm_SBS.stopIntake()));

    Globals.controller.rightStick().onTrue(new InstantCommand(() -> Globals.drive_SBS.changeDrive()));
    Globals.controller.leftStick().onTrue(new InstantCommand(() -> Globals.drive_SBS.resetGyro()));

    Globals.controller.x().onTrue(Globals.arm_SBS.setArm(Globals.armMode.REEF));
    Globals.controller.a().onTrue(Globals.arm_SBS.setArm(Globals.armMode.ALGAE));
    Globals.controller.b().onTrue(Globals.arm_SBS.setArm(Globals.armMode.CORAL));
    Globals.controller.y().onTrue(Globals.arm_SBS.setArm(Globals.armMode.HOME));


    Globals.controller.start().onTrue(Globals.arm_SBS.setArm(Globals.armMode.CLIMB));
    Globals.controller.back().onTrue(new InstantCommand(() -> Globals.arm_SBS.lockArm()));

    Globals.controller.povLeft().onTrue(Globals.arm_SBS.setArm(Globals.POV.LEFT));
    Globals.controller.povRight().onTrue(Globals.arm_SBS.setArm(Globals.POV.RIGHT));
    Globals.controller.povUp().onTrue(Globals.arm_SBS.setArm(Globals.POV.UP));
    Globals.controller.povDown().onTrue(Globals.arm_SBS.setArm(Globals.POV.DOWN));
  }

  public Command getAutonomousCommand() {
    return Globals.autonomous_command.getSelected();
  }
}
