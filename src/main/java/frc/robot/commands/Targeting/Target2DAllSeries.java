// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Target2DAllSeries extends SequentialCommandGroup {
  /** Creates a new Target2DAllSeries. */
  public Target2DAllSeries(Swerve s_Swerve) {
    addCommands(
    new Target2DAngle(s_Swerve,  0,0).withTimeout(1),
    new Target2DForwardDistance(s_Swerve, 0, 0, 12).withTimeout(1),
    new Target2DSideDistance(s_Swerve, 0, 0, 12).withTimeout(1));
  }
}
