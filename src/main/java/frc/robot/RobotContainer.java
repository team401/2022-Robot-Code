package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.Climber.ExtendTelescope;
import frc.robot.commands.Climber.RetractTelescope;
import frc.robot.commands.Climber.UpdateRotationArm;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.drivetrain.RunAtPercent;
import frc.robot.commands.superstructure.HoodCalibrate;
import frc.robot.commands.superstructure.HoodToSetPoint;
import frc.robot.commands.superstructure.turret.manual.ManualTurret;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  //initialize the two joysticks (left and right) and our gamepad
  private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
  private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);
  private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);


  //list out all of the subsystems we need in our robot
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final VisionSubsystem limelightSubsystem = new VisionSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {

    //sets the default command of the drive to Operator Control (normal teleop driving) 
    //this set uses the gamepad as input for easy testing on field
    //for more info, go to the command
    /*drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> gamepad.getLeftY(), //Left up & down
        () -> gamepad.getLeftX(), //Left side-to-side
        () -> gamepad.getRightX(), //Right side-to-side
        true
      )
    );*/
    
    //uncomment this to use the competition control set-up (with two joysticks rather than the gamepad)
    /*drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> leftJoystick.getY(), //Left up & down
        () -> leftJoystick.getX(), //Left side-to-side
        () -> rightJoystick.getX(), //Right side-to-side
        true
      )
    );*/

    SmartDashboard.putNumber("Hood SetPoint", 0);

    configureButtonBindings();

  }

  //where we put all of our button bindings
  private void configureButtonBindings() {

    /*
    Gamepad
      Y
    X   B
      A
    */

    //for testing
    //runs both the drive and rotation motors at a set speed to make sure they are all working
    //new JoystickButton(gamepad, Button.kX.value)
      //.whileHeld(new RunAtPercent(drive));

        //Climber Button Bindings ***TEMPORARY***
    /**
     * A = Intake Arm Position
     * B = Climb Arm Position
     * C = Default Arm Position
     * 
     * Left Xbox Trigger = Extend Telescope
     * Right Xbox Trigger = Retract Telescope
     */

    /*new JoystickButton(gamepad, Button.kA.value)
      .whenPressed(new UpdateRotationArm(climb, ClimberConstants.intakeArmPosition));

    new JoystickButton(gamepad, Button.kB.value)
      .whenPressed(new UpdateRotationArm(climb, ClimberConstants.climbArmPosition));

    new JoystickButton(gamepad, Button.kX.value)
      .whenPressed(new UpdateRotationArm(climb, ClimberCons tants.defaultArmPosition));

    new JoystickButton(gamepad, Button.kLeftBumper.value)
      .whenPressed(new ExtendTelescope(climb));

    new JoystickButton(gamepad, Button.kRightBumper.value)
      .whenPressed(new RetractTelescope(climb));*/

    new JoystickButton(rightJoystick, 3)//Button.kA.value)
      .whenPressed(new InstantCommand(shooterSubsystem::runHood))
      .whenReleased(new InstantCommand(shooterSubsystem::stopHood));

    new JoystickButton(rightJoystick, 2)//Button.kX.valurightJoye)
      .whenHeld(new HoodCalibrate(shooterSubsystem));
      //.whenReleased(new InstantCommand(shooter::stopHood));

    new JoystickButton(rightJoystick, 5)
      .whenPressed(new HoodToSetPoint(shooterSubsystem));

    //whenReleased sets the command to be interruptable, so they should stop if button is pressed/released
    /*new JoystickButton(rightJoystick, 3)
      .whenHeld(new Tracking(limelight, turret))
      .whenReleased(new ManualTurret(
          limelight, 
          turret, 
          () -> turretManualLeftButton.get(), 
          () -> turretManualRightButton.get()
        )
      );*/

  }

  //prepares the robot for autonomous and send sthe command we should use
  public Command getAutonomousCommand() {
   
    return null;

  }
}
