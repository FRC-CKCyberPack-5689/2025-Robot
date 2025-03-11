// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autocmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.RMap.Globals;
import frc.robot.RMap.Globals.POV;
import frc.robot.RMap.Globals.armMode;
import frc.robot.RMap.SpeedConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleReefAuto extends SequentialCommandGroup {
  /** Creates a new MiddleReefAuto. */
  public MiddleReefAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Globals.arm_SBS.setArm(armMode.REEF),
      Globals.drive_SBS.autoDriveX(3,-0.2),
      new ParallelCommandGroup(Globals.drive_SBS.autoDriveX(1, 0.1), Globals.arm_SBS.AutoIntake(SpeedConstants.kCORAL_SHOOT, 2.0)),
      Globals.arm_SBS.setArm(armMode.ALGAE),
      Globals.drive_SBS.autoDriveX(1, -0.2),
      new StartEndCommand(() -> Globals.arm_SBS.setIntakeSpeed(SpeedConstants.kBALL_INTAKE),() -> Globals.arm_SBS.stopIntake()).withTimeout(2.0),
      Globals.drive_SBS.autoDriveX(0.5, 0.2),
      Globals.drive_SBS.autoRotate(Globals.gyro.getAngle()-270),
      Globals.arm_SBS.setArm(POV.AUTO1)
      );
  }
}
