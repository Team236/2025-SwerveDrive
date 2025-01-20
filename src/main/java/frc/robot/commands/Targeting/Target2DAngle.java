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
public class Target2DAngle extends Command {
    // simple proportional turning control with Limelight.
   // "proportional control" is a control algorithm in which the output is proportional to the error.
   // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kProtation = 0.008;
    private double pipeline = 0; 
    private double tv, angleTx;
    private double translationSup, strafeSup;
    private Swerve s_Swerve;    
  
  /** Creates a new Target2DAngle. */
  public Target2DAngle(Swerve s_Swerve, double translationSup, double strafeSup) {
    this.s_Swerve = s_Swerve;
    this.strafeSup = strafeSup;
    this.translationSup = translationSup;
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
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees  (tx is the angle from the target, i.e. angle error)
    angleTx = LimelightHelpers.getTX("limelight");
    SmartDashboard.putNumber("TargetingAngle: ", angleTx);

    double targetingAngle = angleTx * kProtation; //
    // convert to radians per second for our drive method
    
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngle *= -1.0; 
 
    double rotationVal = targetingAngle; 

    //This sets Forward and Sideways movement equal to the value passed when command called (which is joystick value)
    // or try translationVal and strafeVal = 0 if needed (no movement in X or Y directions)
    double translationVal = MathUtil.applyDeadband(translationSup, Constants.stickDeadband);
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
