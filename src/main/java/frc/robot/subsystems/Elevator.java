// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax leftElevatorMotor, rightElevatorMotor;
  private SparkMaxConfig leftConfig, rightConfig;

  public RelativeEncoder leftElevEncoder, rightElevEncoder;

  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isTException, isBException;

  public Elevator() {
    leftElevatorMotor = new SparkMax(Constants.MotorControllers.ID_ELEVATOR_LEFT, MotorType.kBrushless);
    rightElevatorMotor = new SparkMax(Constants.MotorControllers.ID_ELEVATOR_RIGHT, MotorType.kBrushless);

   // leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // configure motors
    leftConfig = new SparkMaxConfig();
    leftConfig.inverted(true);
    leftConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    rightConfig = new SparkMaxConfig();
    rightConfig.inverted(false);
    rightConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    leftElevEncoder = leftElevatorMotor.getEncoder();
    rightElevEncoder = rightElevatorMotor.getEncoder();

    try {
      elevatorTopLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_TOP);
    } catch (Exception e) {
      isTException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isTException);
    }

    try {
      elevatorBottomLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_BOTTOM);
    } catch (Exception e) {
      isBException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isBException);
    }

  }

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }
  public boolean isETopLimit() {
    if (isTException) {
      return true;
    } else {
      return elevatorTopLimit.get();
    }
  }
  public boolean isEBotLimit() {
    if (isBException) {
      return true;
    } else {
      return elevatorBottomLimit.get();
    }
  }

  // reset/zero encoders
  public void resetElevatorEncoders(){
    leftElevEncoder.setPosition(0);
    rightElevEncoder.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS (number of rotations)
  public double getElevLeftEncoder() {
    return leftElevEncoder.getPosition(); //for a SparkMax encoder
  }
    public double getElevRightEncoder() {
    return rightElevEncoder.getPosition(); //for a SparkMax encoder
  }

  // currently returns revolutions of encoders -- NOT CONVERTED TO IN.
  public double getElevatorHeight(){
    return Constants.Elevator.ELEV_REV_TO_IN * ((getElevLeftEncoder() + getElevRightEncoder()) / 2);
  }

  // check if elevator is at "top" according to user definition
  public boolean isTop() {
    boolean eTop;
    if (getElevatorHeight() >= Constants.Elevator.MAX_HEIGHT) {
      eTop = true;
    } else {
      eTop = false;
    }
    return eTop;
  }

  public void setElevSpeed(double speed) {
    if (speed > 0) {  
      if (isETopLimit() || isTop()) {
          // if elevator limit is tripped or elevator is near the top limit switch going up, stop 
          stopElevator();
       }  else {
          // elevator going up but top limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
      } 
    else {
      if (isEBotLimit()) {
        //elevator going down and is at the bottom,stop and zero encoder
        stopElevator();
        resetElevatorEncoders();
      } else {
      // elevator going down but not at the bottom, go at commanded speed
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
      }
    }
  } 

  public double getElevatorLeftSpeed() {
    return leftElevatorMotor.get();
  }  

  public double getElevatorRightSpeed() {
    return rightElevatorMotor.get();
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator height: ", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator at top? ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom? ", isEBotLimit());
  }
}
