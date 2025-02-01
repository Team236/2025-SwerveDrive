// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  public UsbCamera usbCamera0;

  // Auto starting Trajectories located in deploy/pathplanner/ autos & paths
  private static String BlueOneJsonPath = "paths//BlueOne.wpilib.json";
  private static String BlueTwoJsonPath60a = "paths//BlueTwo.wpilib.json";
  private static String BlueThreeJsonPath = "paths//BlueThree.wpilib.json";
  // Auto secondary Trajectories
  private static String ToCoral_60aJsonPath = "paths//PickupFromCoral60a.wpilib.json";
  private static String TwoJsonPath = "paths//Coral60bFromCoral.json";
  private static String ThreeJsonPath = "paths//PickupFromCoral60b.json";
  private static String FourJsonPath = "paths//Coral60aFromCoral.wpilib.json";
 
  
  public static Trajectory blue1Trajectory1= new Trajectory();  // Blue1 to reef 60a
  public static Trajectory traj2= new Trajectory();  // go to coral pickup from 60a
  public static Trajectory traj3= new Trajectory();  // back to reef 60b
  public static Trajectory traj4= new Trajectory();  // go to coral pickup from 60b
  public static Trajectory traj5= new Trajectory();  // back to reef 60a 

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public Field2d field;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    readTrajectories();
    
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        
      //USB camera
      try {
        usbCamera0 = CameraServer.startAutomaticCapture(0);
        } catch (Exception e)  {
       SmartDashboard.putString("camera capture failed", "failed");
        }
    
    //field 2d allows us to visualize an auto trajectory on a dashboard even without the robot using simulation, here the configs are set
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
    //trajectory is created, pose2d for start and finish with translation 2d for points to hit in between
        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(2, 0, new Rotation2d(0)),
                    List.of(new Translation2d(2, 0.25 ), new Translation2d(2, 0.5), new Translation2d(2,0.75), new Translation2d(2,1),
                    new Translation2d(2,1.25), new Translation2d(2,1.5), new Translation2d(2,1.75), new Translation2d(2,2),
                    new Translation2d(2,2.25), new Translation2d(2,2.5), new Translation2d(2,2.75), new Translation2d(2,3),
                    new Translation2d(2,3.25), new Translation2d(2,3.5), new Translation2d(2,3.75)),
                    new Pose2d(2, 4, new Rotation2d(0)),
                    config);
    
        field = new Field2d();
        SmartDashboard.putData(field);
        
        
        field.getObject("trajectory").setTrajectory(trajectory);
        field.getObject("PIPose2d").setPose(new Pose2d(2,3,new Rotation2d(Math.PI)));
        field.getObject("zero Pose2d").setPose(new Pose2d(5,3,new Rotation2d(0)));
        //Need to do this once in order to have Limelight communication while tethered
        for (int port = 5800; port <= 5805; port++){
          PortForwarder.add(port, "limelight.local", port);
        }
      }
    
      private void readTrajectories() {
      // TODO read the trajectories from the file system
        //paths = "paths//";
        String BlueOnetoCoral60a = "paths//bluexxxxx.wpilib.json";  
        
         try {
           Path BlueOnePath1 = Filesystem.getDeployDirectory().toPath().resolve(BlueOnetoCoral60a);

           blue1Trajectory1 = TrajectoryUtil.fromPathweaverJson(Paths.get(BlueOnetoCoral60a));
         } catch (IOException e) {
           DriverStation.reportError("Unable to open trajectory: " + BlueOnetoCoral60a, false);
         }
        //throw new UnsupportedOperationException("Unimplemented method 'readTrajectories'");
      }
    
      /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    LimelightHelpers.setCropWindow("limelight", -.5, .7, -1, .9);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
