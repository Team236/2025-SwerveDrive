// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Target2DDistance extends Command {
// simple ranging control with Limelight.

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
    private double h1 = 30 * 0.0254; // meters, from ground to center of camera lens
    private double a1 = Math.toRadians(21); //20 degrees, camera tilt
    private double targetHeight = 12* 0.0254;  //meters distance from floor to center of target
    private double standoff; // desired horiz distance from camera to target in meters; pass into command
    private double disY, a2, dx, errorY;

  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to set angular velocity that is proportional to the 
  // "tx" value (anlge between the LL and the target) from the Limelight.
  //and forward speed will be proportional to the "ty" value, which is the forward distance to the target

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    double kProtation = 0.035;
    double kPtranslation = 0.1;
    private double pipeline = 0; 
    private double tv;
    private double strafeSup, rotationSup; 
    private Swerve s_Swerve;    
  
  /** Creates a new Target2DAngleDistance. */
  public Target2DDistance(Swerve s_Swerve, double strafeSup, double rotationSup, double standoff) {
    this.s_Swerve = s_Swerve;
    this.strafeSup = strafeSup;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);


    if (tv ==1) { //tv =1 means Limelight sees a target

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty" 
   
      //NOTE:  CAN TRY TO USE THE Z VALUE OF THE POSE FOR errorY (use [2] or [0] for other directions)
     //double errorY = NetworkTableInstance.getDefault().getTable("limelight").
    //getIntegerTopic("targetpose_cameraspace").subscribe(new double[]{}).get()[3];

/*  double ty = LimelightHelpers.getTY("limelight");
    disY = Math.abs(ty);  //vertical offset from crosshair to target in degrees
    a2 = disY*Math.PI/180;// in radians, since disY in degrees
    dx = Math.abs(targetHeight - h1)/Math.tan(a1+a2);
    errorY = standoff - dx;
    // double errorY = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
   
    double targetingForwardSpeed = kPtranslation * errorY;
*/
    SmartDashboard.putNumber("Limelight errorY in meters", errorY);
    double targetingForwardSpeed = (LimelightHelpers.getTY("limelight"))* kPtranslation;
   
    targetingForwardSpeed *= -1.0;
    double translationVal = targetingForwardSpeed;
   
   //This sets Y and rotational movement equal to the value passed when command called (which is joystick value)
   // or try strafeVal and rotationVal = 0 if needed (no rotation or movement in Y directions)
   double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.stickDeadband);
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
