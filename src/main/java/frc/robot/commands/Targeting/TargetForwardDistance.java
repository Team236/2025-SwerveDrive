// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Targeting;

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
public class TargetForwardDistance extends Command {
// simple ranging control with Limelight.
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

  private double standoff; //desired Forward distance in inches from bumper to tag; pass into command
  private double dz, error; //z is the forward direction

  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to set forward speed that is proportional to the forward
  // distance between the target and the robot frame

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    double kPtranslation = 0.4; //kP for the Forward direction
    private double pipeline = 0; 
    private double tv;
    private double strafeSup, rotationSup; 
    private Swerve s_Swerve;    
  
  /** Creates a new TargetForwardDistance. */
  public TargetForwardDistance(Swerve s_Swerve, double strafeSup, double rotationSup, double standoff) {
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
  // simple proportional ranging control
  // this works best if your Limelight's mount height and target mount height are different.
    dz = LimelightHelpers.getTargetPose_RobotSpace("limelight")[2]; //Fwd dist from center of robot to target 
  //Add the forward dist from bumper to center of robot (from Constants) to the desired standoff from the bumper:
    double finalStandoff = (standoff + Constants.Targeting.DIST_TO_CENTER) * 0.0254; //to robot center in meters
    error = dz - finalStandoff; 
    double targetingForwardSpeed = error*kPtranslation;

     SmartDashboard.putNumber("Forward distance from Robot Bumper to tag in inches: ", ((dz/0.0254)-Constants.Targeting.DIST_TO_CENTER));
    //targetingForwardSpeed *= -1.0;
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
