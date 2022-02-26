package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.climber.CalibrateTelescope;
import frc.robot.commands.climber.ClimbSequence;
import frc.robot.commands.climber.UpdateTelescopeArms;
import frc.robot.commands.climber.HoldPositionRotationArms;
import frc.robot.commands.climber.UpdateRotationArm;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.drivetrain.RunAtPercent;
import frc.robot.commands.superstructure.ballHandling.ReverseIndexing;
import frc.robot.commands.superstructure.ballHandling.Intake;
import frc.robot.commands.superstructure.shooting.HoodCalibrate;
import frc.robot.commands.superstructure.shooting.HoodToSetPoint;
import frc.robot.commands.superstructure.shooting.PrepareToShoot;
import frc.robot.commands.superstructure.shooting.Shoot;
import frc.robot.commands.superstructure.turret.limelight.Tracking;
import frc.robot.commands.superstructure.turret.manual.ManualTurret;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

  //initialize the two joysticks (left and right) and our gamepad
  private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
  private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);
  private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);


  //list out all of the subsystems we need in our robot
  private final DriveSubsystem drive = new DriveSubsystem();
  private final TelescopeArmSubsystem telescopeArmSubsystem = new TelescopeArmSubsystem();
  private final RotationArmSubsystem rotationArmSubsystem = new RotationArmSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IndexingSubsystem indexingSubsystem = new IndexingSubsystem();

  public RobotContainer() {

    // Sets the default command of the drive to Operator Control (normal teleop driving) 
    // This set uses the gamepad as input for easy testing on field
    // For more info, go to the command
    /*drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> gamepad.getLeftY(), //Left up & down
        () -> gamepad.getLeftX(), //Left side-to-side
        () -> gamepad.getRightX(), //Right side-to-side
        true
      )
    );*/
    
    // Uncomment this to use the competition control set-up (with two joysticks rather than the gamepad)
    drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> leftJoystick.getY(), // Left up & down
        () -> leftJoystick.getX(), // Left side-to-side
        () -> rightJoystick.getX(), // Right side-to-side
        true
      )
    );

    // Commented out for testing, uncomment for competition
    //shooterSubsystem.setDefaultCommand(new Tracking(limelightSubsystem, turretSubsystem));

    SmartDashboard.putNumber("Hood SetPoint", 0);

    configureButtonBindings();

  }

  // Where we put all of our button bindings
  private void configureButtonBindings() {

    // Button Bindings Overview
    /**
     * GAMEPAD:
     * 
     * X = Climb sequence
     * TODO: Y = Shoot
     * B = Deploy rotation arms & Intake and Index
     * A = Nothing
     * 
     * Left Trigger = Nothing
     * TODO: Right Trigger = Ramp Up
     * 
     * Left Bumper = Telescope to 32 inches
     * Right Bumper = Telescope to 5 inches
     * 
     * Start = Calibrate Telescope
     * Back = Deploy rotation arms & Reverse Intake and Index
     * 
     * POV Up = Rotation Arm to Straight Up (Default Position)
     * POV Right = Rotation Arm to Climb Position
     * POV Down = Rotation Arm to Intake Position
     * POV Left = Rotation Arm to Back Position
     * 
     * 
     * JOYSTICK:
     * 
     * Left = Drive
     * Right = Rotate
     * 
     * Right 2 = Calibrate Field Relative
     * 
     */

    
    // Rotation Arms
    new POVButton(gamepad, 0)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.defaultArmPosition, 
        new TrapezoidProfile.Constraints(10.0, 15.0))
        .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new POVButton(gamepad, 90)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.climbArmPosition, 
      new TrapezoidProfile.Constraints(10.0, 15.0))
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new POVButton(gamepad, 180)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.intakeArmPosition, 
        new TrapezoidProfile.Constraints(10.0, 15.0))
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new POVButton(gamepad, 270)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.backArmPosition,
        new TrapezoidProfile.Constraints(10.0, 15.0))
        .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));
    

    // Field Relative Reset
    new JoystickButton(rightJoystick, 2)
      .whenPressed(() -> drive.resetIMU());
    

    // Telescope Arms
    new JoystickButton(gamepad, Button.kLeftBumper.value)//Axis.kLeftTrigger.value)
      .whenHeld(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(0.25))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(0.25))))
      .whenReleased(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(0.0))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(0.0))));

    new JoystickButton(gamepad, Button.kRightBumper.value)//Axis.kRightTrigger.value)
      .whenHeld(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(-0.25))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(-0.25))))
      .whenReleased(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(0.0))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(0.0))));

    new JoystickButton(gamepad, Button.kStart.value)
      .whenPressed(new CalibrateTelescope(telescopeArmSubsystem));
    
    /*new JoystickButton(gamepad, Button.kLeftBumper.value)
      .whenPressed(new UpdateTelescopeArms(climbSubsystem, turretSubsystem, 32.0));

    new JoystickButton(gamepad, Button.kRightBumper.value)
      .whenPressed(new UpdateTelescopeArms(climbSubsystem, turretSubsystem, 5.0));*/


    // Intake and Index
    /*new JoystickButton(gamepad, Button.kB.value)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.defaultArmPosition, 
        new TrapezoidProfile.Constraints(10.0, 15.0))
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)))
      .whenHeld(new Intake(indexingSubsystem, intakeSubsystem));*/
    
    new JoystickButton(gamepad, Button.kBack.value)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.defaultArmPosition, 
        new TrapezoidProfile.Constraints(10.0, 15.0))
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)))
      .whenHeld(new ReverseIndexing(indexingSubsystem, intakeSubsystem));


    // Shooting
    new JoystickButton(gamepad, Button.kY.value)
      .whenPressed(new PrepareToShoot(shooterSubsystem, limelightSubsystem, 4000));

    new JoystickButton(gamepad, Button.kA.value)
      .whenHeld(new InstantCommand(
        () -> shooterSubsystem.runShooterVelocityController(SmartDashboard.getNumber("Speed", 0))));
      
    new JoystickButton(gamepad, Button.kB.value)
      .whenHeld(new InstantCommand(
        () -> shooterSubsystem.hoodSetDesiredClosedStateRevolutions(SmartDashboard.getNumber("Position", 0))));

    // Climb Sequence
    new JoystickButton(gamepad, Button.kX.value)
      .whenPressed(new ClimbSequence(telescopeArmSubsystem, rotationArmSubsystem, turretSubsystem));

  }

  // Prepares the robot for autonomous and sends the command we should use
  public Command getAutonomousCommand() {
   
    return null;

  }
}
