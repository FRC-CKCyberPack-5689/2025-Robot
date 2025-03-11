// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autocmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RMap.Globals;
import frc.robot.RMap.Globals.POV;
import frc.robot.RMap.Globals.armMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testAutoAlgaeCamView extends SequentialCommandGroup {
  /** Creates a new testAutoAlgaeCamView. */
  public testAutoAlgaeCamView() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Globals.arm_SBS.setArm(armMode.ALGAE),Globals.arm_SBS.setArm(POV.AUTO1));
  }
}
