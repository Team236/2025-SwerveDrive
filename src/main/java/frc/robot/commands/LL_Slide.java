// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LL_Slide extends Command {

private Swerve swerve;
private double Kp = 0.1;


  /** Creates a new LL_Slide. */
  public LL_Slide(Swerve s_Swerve) {
     swerve = s_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  //TODO verify the limelighthelpers data should use the x-horizontal relative to robot
    // determine the target-to-robot matrix as [x, y, z, yaw, pitch, roll]
      // x is the horizontal distance to the target - that we care about
      // y is the forward distance to the target
      // z is the vertical distance to the target    
    double[] tar_robotMatrix = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    // frc.robot.smartField.setRobotPose(new Pose2d (tar_robotMatrix);

    double x_feed = tar_robotMatrix[0] * Kp;
    
 //TODO verify the translation2d x directionis horizontal left right relative to robot
    Translation2d translation2d= new Translation2d(-x_feed, 0.0);
    swerve.drive(translation2d, 0.0, true, false);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
