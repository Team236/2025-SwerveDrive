// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralPivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coralPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDCoralPivot extends Command {
  /** Creates a new PIDCoralPivot. */
  private coralPivot coralPivot;
    
  private final PIDController pidController;
  private double kP = Constants.AlgaePivot.KP;
  private double kI = Constants.AlgaePivot.KI;
  private double kD = Constants.AlgaePivot.KD;

  public PIDCoralPivot(coralPivot coralPivot, double revs) {

    pidController = new PIDController(kP, kI, kD);
    this.coralPivot = coralPivot;
    addRequirements(this.coralPivot);

    pidController.setSetpoint(revs);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralPivot.setCoralPivotSpeed(pidController.calculate(coralPivot.getCoralEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralPivot.stopCoralPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
