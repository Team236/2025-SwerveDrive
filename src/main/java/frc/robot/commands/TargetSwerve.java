// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetSwerve extends Command {
    // simple proportional turning control with Limelight.
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
    private double offset = 0;  //how far to be from the target, in the forward direction
    private Swerve t_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
  
  /** Creates a new LimelightAimAndRange. */
  public TargetSwerve(Swerve t_swerve, double pipeline, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.pipeline = pipeline;
    this.t_Swerve = t_Swerve;
    addRequirements(t_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;  
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

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees  (tx is the angle from the target, i.e. angle error)
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kProtation;
    // convert to radians per second for our drive method
    
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;  // //LIKELY NEED TO KEEP

    //final var rot_limelight = targetingAngularVelocity;  ///rotation axis
    // rot = rot_limelight;
    double rotationVal = targetingAngularVelocity; 

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty" 
    double targetingForwardSpeed = (LimelightHelpers.getTY("limelight") - offset)* kPtranslation;
    targetingForwardSpeed *= -1.0;
  
   // final var forward_limelight = targetingForwardSpeed;// translation axis
    //xSpeed = forward_limelight;
    double translationVal = targetingForwardSpeed;

    //double strafeVal = 0;  //for now - can we find something that gets the strafe distance from limelight?

    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

   /* Drive */
   t_Swerve.drive(
       new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
       rotationVal * Constants.Swerve.maxAngularVelocity, 
       !robotCentricSup.getAsBoolean(), 
       true
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
