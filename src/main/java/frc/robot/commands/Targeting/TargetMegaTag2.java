// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;


/* WPILib includes 'pose estimators' for differential, swerve and mecanum drivetrains.
*  These estimators are designed to be drop-in replacements for the existing 'odometry' classes
*
*   You should consider using the more terse Command factories API instead 
*   https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands 
*/
// this COMMAND only controls update of the pose estimator information in network tables 
// based on the vision data, when the robot sees multiple targets with Megatag options one or two
// this command does not drive the robot in the docs it is shown on it's own, but in example code it's defind in Drive.java
// note: those examples are often not defined in true subsystem command structure

public class TargetMegaTag2 extends Command {
  private Swerve s_Swerve;  

  /** Creates a new Target3DMegaTag2. */
  public TargetMegaTag2(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
   
    // evaluating which Megatag one or two to use based on above boolean value and 
    // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7) { doRejectUpdate = true; }
        if(mt1.rawFiducials[0].distToCamera > 3) { doRejectUpdate = true; }
      }
      if(mt1.tagCount == 0) { doRejectUpdate = true; }
      if(!doRejectUpdate) {     // if doRejectUpdate is false (or NOT true), then update the pose estimator
        s_Swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        s_Swerve.m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {   // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
      LimelightHelpers.SetRobotOrientation("limelight", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(s_Swerve.gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)   // if doRejectUpdate is false (or NOT true), then update the pose estimator
      {
        s_Swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        s_Swerve.m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
  }
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
