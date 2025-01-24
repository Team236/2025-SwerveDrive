package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Targeting.TargetMegaTag2;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.commands.Targeting.TargetAllSeries;
import frc.robot.commands.Targeting.TargetAngle;
import frc.robot.commands.Targeting.TargetForwardDistance;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    XboxController driver = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
    XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

  //AUTO SWITCHES
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.Swerve.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.Swerve.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.Swerve.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.Swerve.DIO_AUTO_4);

   //create instance of each subsystem  
   private final Swerve s_Swerve = new Swerve();

    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    //DRIVE COMMANDS
    //***Forward standoff - input into command as inches from bumper
    private final TargetAllParallel targetAllParallel = new TargetAllParallel(s_Swerve, 12, 0);
    private final TargetAllSeries targetAllSeries = new TargetAllSeries(s_Swerve, 12, 0);
    private final TargetAngle targetAngle =  new TargetAngle(s_Swerve, -driver.getRawAxis(translationAxis), -driver.getRawAxis(strafeAxis));
    private final TargetForwardDistance targetForwardDistance = new TargetForwardDistance(s_Swerve, -driver.getRawAxis(strafeAxis), -driver.getRawAxis(rotationAxis), 12);
    private final TargetSideDistance targetSideDistance = new TargetSideDistance(s_Swerve, -driver.getRawAxis(translationAxis), -driver.getRawAxis(rotationAxis), 0);
    private final TargetMegaTag2 target3DMegaTag2 = new TargetMegaTag2(s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        
 // XBOXCONTROLLER - DRIVER CONTROLLER
    JoystickButton x = new JoystickButton(driver, Constants.XboxController.X);
    JoystickButton a = new JoystickButton(driver, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driver, Constants.XboxController.B);
   //Y on driver controller is assigned to "zeroGyro" above
   // JoystickButton y = new JoystickButton(driver, Constants.XboxController.Y);
   //leftBumper on driver controller is assigned to "robotCentric" above
   // JoystickButton lb = new JoystickButton(driver, Constants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driver, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driver, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driver, Constants.XboxController.RM);
    JoystickButton view = new JoystickButton(driver, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driver, Constants.XboxController.MENU);
    POVButton upPov = new POVButton(driver,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driver,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driver,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driver,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    // XBOX CONTROLLER - AUX CONTROLLER
    JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);
    JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);
    JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE);
    POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);

    //y button is already assigned to ZeroGyro
    //leftBumper button is already assigned to RobotCentric
    a.whileTrue(targetAllSeries);
    b.whileTrue(targetAllParallel);
    lm.whileTrue(targetAngle);
    x.whileTrue(targetForwardDistance);
    rb.whileTrue(targetSideDistance);
    rm.whileTrue(target3DMegaTag2);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, false);
        
    }
}
