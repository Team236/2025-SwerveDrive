// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Target2DAllParallel extends Command {
// Basic targeting data
//tx =  Horizontal offset from crosshair to target in degrees
//ty = Vertical offset from crosshair to target in degrees
//ta = Target area (0% to 100% of image)
//tv = hasTarget, Do you have a valid target?
        // 3D Pose Data
        //.getRobotPose_FieldSpace();    // Robot's pose in field space
        //.getCameraPose_TargetSpace();   // Camera's pose relative to tag
        // .getRobotPose_TargetSpace();     // Robot's pose relative to tag
        // .getTargetPose_CameraSpace();   // Tag's pose relative to camera
        //.getTargetPose_RobotSpace();     // Tag's pose relative to robot
        // ? 3D pose array contains [0] = X, [1] = Y, [2] = Z, [3] = roll, [4] = pitch, [5] = yaw

        private double standoffForward; // desired Forward distance in inches from bumper to tag; pass into command
        private double standoffSideways; // desired sideways distance in inches from camera to tag; pass into command
        private double error, dz, dx, angleTx;
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to set angular velocity that is proportional to the 
  // "tx" value (horizontal anlge between the LL crosshair and the target) from the Limelight.
  //and forward speed will be proportional to the forward distance between the robot center and the tag

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kProtation = 0.008; //kP value for rotation
    double kPtranslation = 0.4;//kP value for forward (translation) motion
    double kPstrafe = 0.4;  //kP value for the sideways (strafe) motion
    private double pipeline = 0; 
    private double tv;
    
    private Swerve s_Swerve;    
  
  /** Creates a new Target2DAngleDistance. */
  public Target2DAllParallel(Swerve s_Swerve, double standoffForward, double standoffSideways) {
    this.s_Swerve = s_Swerve;
    this.standoffForward = standoffForward;
    this.standoffSideways = standoffSideways;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv == 1) { //tv =1 means Limelight sees a target
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees  (tx is the angle from the target, i.e. angle error)
    angleTx = LimelightHelpers.getTX("limelight");
    SmartDashboard.putNumber("TargetingAngle: ", angleTx);
    double targetingAngle = angleTx * kProtation; 
    // convert to radians per second for our drive method
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngle *= -1.0; 
    double rotationVal = targetingAngle; 



    dz = LimelightHelpers.getTargetPose_RobotSpace("limelight")[2]; //Fwd dist from center of robot to target 
  //Add the forward dist from bumper to center of robot (from Constants) to the desired standoff from the bumper:
    double finalForward = (standoffForward + Constants.Targeting.DIST_TO_CENTER) * 0.0254; //to robot center in meters
    error = dz - finalForward; 
    double targetingForwardSpeed = error*kPtranslation;
    SmartDashboard.putNumber("Forward distance from Robot Bumper to tag in inches: ", ((dz/0.0254)-Constants.Targeting.DIST_TO_CENTER));
    //targetingForwardSpeed *= -1.0;
    double translationVal = targetingForwardSpeed;



    dx = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]; //sideways dist from camera center to tag in meters
    double finalSideways =standoffSideways * 0.0254;  //convert desired standoff from inches to meters
    error = dx - finalSideways; //OR DO WE NEED ADD finalStandoff here instead of subtract it?
    double targetingSidewaysSpeed = error*kPstrafe;
    SmartDashboard.putNumber("Side to side distance - camera to target, in meters: ", dx);
    targetingSidewaysSpeed *= -1.0;  //NEEDED?
    double strafeVal = targetingSidewaysSpeed;


   

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
