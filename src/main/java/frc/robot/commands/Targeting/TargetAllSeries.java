// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetAllSeries extends SequentialCommandGroup {
  /** Creates a new TargetAllSeries. */
  public TargetAllSeries(Swerve s_Swerve, double standoffForward, double standoffSideways) {
    addCommands(
    new TargetAngle(s_Swerve,  0,0).withTimeout(1),
    new TargetForwardDistance(s_Swerve, 0, 0, standoffForward).withTimeout(1),
    new TargetSideDistance(s_Swerve, 0, 0, standoffSideways).withTimeout(1));
  }
}
