// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralPivot extends Command {

  private coralPivot coralPivot2;
  private double coralSpeed;

  /** Creates a new ManualMove. */
  public ManualCoralPivot(coralPivot coralPivot2, double coralSpeed) {
    this.coralPivot2 = coralPivot2;
    this.coralSpeed = coralSpeed;
    addRequirements(this.coralPivot2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralPivot2.setCoralPivotSpeed(coralSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralPivot2.stopCoralPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
