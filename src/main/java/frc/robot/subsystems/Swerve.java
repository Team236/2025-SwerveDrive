package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator m_poseEstimator;
    private Command m_pathCommand;

     // pathPlanner stuff
    public static PathPlannerPath pathPlannerPath;
    private static RobotConfig robotConfig;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        // read the robot configuration from the PathPlanner GUI settings
        try {
            robotConfig = RobotConfig.fromGUISettings();
            }    catch (ParseException e) {
                DriverStation.reportError("IOException: " + e.getMessage(), e.getStackTrace());
            } catch (IOException  e) {
                DriverStation.reportError("ParseException" + e.getMessage(), e.getStackTrace());
            }

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), //front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), //front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), //back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants) //back right
            };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings, for 3D targeting. 
        The numbers used below are robot specific, and should be tuned. */
           m_poseEstimator = new SwerveDrivePoseEstimator(
             Constants.Swerve.swerveKinematics,
              gyro.getRotation2d(),
              new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(), //front left
                mSwerveMods[1].getPosition(), //front right
                mSwerveMods[2].getPosition(), //back left
                mSwerveMods[3].getPosition()  //back right
              },
              new Pose2d(),
              VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), //std deviations in X, Y (meters), and angle of the pose estimate
              VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));  //std deviations  in X, Y (meters) and angle of the vision (LL) measurement
            
            
            }

//Methods start here:

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), 
                rotation, getHeading() )
                : new ChassisSpeeds( translation.getX(), translation.getY(), rotation) );
                
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); //closed loop auto
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    // duplicate to setPose for PathPlanner (not sure if specific method name is required?)
    public void resetPose(Pose2d pose) {  
        this.setPose(pose);
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        //return Rotation2d.fromDegrees(gyro.getYaw().getValue());
       return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        // This update call keeps track of Robot position on field
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder degrees", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle degrees", mod.getPosition().angle.getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity m/s", mod.getState().speedMetersPerSecond);    
        }
    }

    //method Also supporta PathPlanner use
    public ChassisSpeeds getRobotRelativeSpeeds() {
    // Implement the logic to return the current robot-relative speeds
    return new ChassisSpeeds(0, 0, 0); // Replace with actual implementation
}

    // PathPlanner method to follow path specified in the calling of the method from a command class
    public Command followPathCommand(String pathName) {
        Optional<Pose2d> poseArray =RobotContainer.auto1_path1.getStartingHolonomicPose();
        try{

        m_pathCommand = new FollowPathCommand( 
                pathPlannerPath,
                poseArray, //this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0) ),// Translation and Rotation PID constants
                robotConfig, // robot configuration pulled from PathPlanner file
                () -> { return false;  },
                this );     // Reference to this subsystem to set requirements
        

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
        return m_pathCommand;      //  this should return a real command from the new FollowPath code
    }

  // PathPlanner method to drive robot relative
  public void driveRobotRelative(ChassisSpeeds speeds) {
    
  }
}

