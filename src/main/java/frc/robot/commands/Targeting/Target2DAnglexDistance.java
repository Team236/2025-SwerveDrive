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
public class Target2DAnglexDistance extends Command {

  // Basic targeting data
//tx =  Horizontal offset from crosshair to target in degrees
//ty = Vertical offset from crosshair to target in degrees
//ta = Target area (0% to 100% of image)
//tv = hasTarget, Do you have a valid target?
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = ty = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;
          
        // 3D Pose Data
        //.getRobotPose_FieldSpace();    // Robot's pose in field space
        //.getCameraPose_TargetSpace();   // Camera's pose relative to tag
        // .getRobotPose_TargetSpace();     // Robot's pose relative to tag
        // .getTargetPose_CameraSpace();   // Tag's pose relative to camera
        //.getTargetPose_RobotSpace();     // Tag's pose relative to robot
        // ? 3D pose array contains [0] = X, [1] = Y, [2] = Z, [3] = roll, [4] = pitch, [5] = yaw

    private double h1 = 30*0.0254; //meters to inches, from ground to center of camera lens
    private double a1 = Math.toRadians(21); //20 degrees, camera tilt
    private double targetHeight = 12*0.0254;  //distance from floor to center of target, in meters
    private double standoff; // desired distance from camera to target in meters; pass into command
    private double disY, a2, dx, dy, error, myYaw, myTx;


  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to set angular velocity that is proportional to the 
  // "tx" value (horizontal anlge between the LL crosshair and the target) from the Limelight.
  //and forward speed will be proportional to the "ty" value

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kProtation = 0.035;
    double kPtranslation = 0.1;
    private double pipeline = 0; 
    private double tv, strafeSup; 
    
    private Swerve s_Swerve;    
  
  /** Creates a new Target2DAngleDistance. */
  public Target2DAnglexDistance(Swerve s_Swerve, double strafeSup, double standoff) {
    this.s_Swerve = s_Swerve;
    this.strafeSup = strafeSup;
    this.standoff = standoff;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);

    double myYaw = LimelightHelpers.getTargetPose_RobotSpace("limelight")[5];  // horizontal angle - robot to tag
    double myTx = LimelightHelpers.getTX("limelight");
    SmartDashboard.putNumber("myYaw ", myYaw);
    SmartDashboard.putNumber("myTx", myTx);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv == 1) { //tv =1 means Limelight sees a target
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees  (tx is the angle from the target, i.e. angle error)
    //double targetingAngularVelocity = LimelightHelpers.getTargetPose_RobotSpace("limelight")[5]*kProtation;  // horizontal X distance from camera to tag
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kProtation;
    // convert to radians per second for our drive method   ????

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;  // //LIKELY NEED TO KEEP

    double rotationVal = targetingAngularVelocity; 

  // simple proportional ranging control with Limelight's "ty" value (vertial offset from LL crosshair to target, in degrees)
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty" 
   //???? DO WE NEED TO CONVERT THE ty INTO DISTANCE?  AS DONE LAST YEAR?
   
   
    //double ty = LimelightHelpers.getTY("limelight");
    //disY = Math.abs(ty);  //vertical offset from crosshair to target in degrees
    //a2 = disY*Math.PI/180;// in radians, since disY in degrees
    //dx = Math.abs(targetHeight - h1)/Math.tan(a1+a2); //horizotal distance to target,meters
    dx = LimelightHelpers.getTargetPose_RobotSpace("limelight")[0];  // horizontal X distance from camera to tag
    dy = LimelightHelpers.getTargetPose_RobotSpace("limelight")[1];  // horizontal Y distance from camera to tag
    error = dx - standoff; 
    double targetingForwardSpeed = error*kPtranslation;
    //double targetingForwardSpeed = (LimelightHelpers.getTY("limelight"))* kPtranslation;
     SmartDashboard.putNumber("Forward X distance - LL camera to target, in meters: ", dx);
     SmartDashboard.putNumber("Sideways Y distance - LL camera to target, in meters: ", dy);

    targetingForwardSpeed *= -1.0;
    double translationVal = targetingForwardSpeed;

   //This sets Y movement equal to the value passed when command called (which is joystick value)
   // or try strafeVal = 0 if needed (no movement in Y directions)
   double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.stickDeadband);
   
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
