// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Target2DSideDistance extends Command {
// simple ranging control with Limelight.

    private double standoff; // desired horiz distance in inches from camera to target; pass into command
    private double dx, error;

// Basic targeting data
//tv = hasTarget, Do you have a valid target?
        // 3D Pose Data
        //.getRobotPose_FieldSpace();    // Robot's pose in field space
        //.getCameraPose_TargetSpace();   // Camera's pose relative to tag
        // .getRobotPose_TargetSpace();     // Robot's pose relative to tag
        // .getTargetPose_CameraSpace();   // Tag's pose relative to camera
        //.getTargetPose_RobotSpace();     // Tag's pose relative to robot
        // ? 3D pose array contains [0] = X, [1] = Y, [2] = Z, [3] = roll, [4] = pitch, [5] = yaw

  // "proportional control" is a control algorithm in which the output is proportional to the error.

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    double kPstrafe = 0.4;
    private double pipeline = 0; 
    private double tv;
    private double translationSup, rotationSup; 
    private Swerve s_Swerve;    
  
  /** Creates a new Target2DAngleDistance. */
  public Target2DSideDistance(Swerve s_Swerve, double translationSup, double rotationSup, double standoff) {
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.rotationSup = rotationSup;
    this.standoff = standoff;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);

    // TODO swap to LimelightHelpers alternative instead of above methods ?
    // LimelightHelpers.setLEDMode_ForceOn("limelight");
    // LimelightHelpers.setPipelineIndex("limelight", pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv ==1) { //tv =1 means Limelight sees a target

    dx = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]; //side to side X dist from camera center to tag
    double finalStandoff = standoff * 0.0254;
    error = dx - finalStandoff; 
    double targetingSidewaysSpeed = error*kPstrafe;

    SmartDashboard.putNumber("Side to side distance - camera to target, in meters: ", dx);

    targetingSidewaysSpeed *= -1.0;  //NEEDED??
    double strafeVal = targetingSidewaysSpeed;
   
   //This sets Y and rotational movement equal to the value passed when command called (which is joystick value)
   // or try strafeVal and rotationVal = 0 if needed (no rotation or movement in Y directions)
   double translationVal = MathUtil.applyDeadband(translationSup, Constants.stickDeadband);
   double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.stickDeadband);
   
   /* Drive */
   s_Swerve.drive(
       new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
       rotationVal * Constants.Swerve.maxAngularVelocity, 
       true,  //true for robot centric
       true //true for open loop (?)
   );
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
