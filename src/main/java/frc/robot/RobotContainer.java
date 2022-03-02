package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.climber.CalibrateTelescope;
import frc.robot.commands.climber.ClimbSequence;
import frc.robot.commands.climber.DefaultRotationArm;
import frc.robot.commands.climber.HoldPositionRotationArms;
import frc.robot.commands.climber.UpdateRotationArm;
import frc.robot.commands.climber.UpdateTelescopeArms;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.superstructure.ballHandling.Intake;
import frc.robot.commands.superstructure.ballHandling.ReverseIndexing;
import frc.robot.commands.superstructure.shooting.HoodCalibrate;
import frc.robot.commands.superstructure.shooting.PrepareToShoot;
import frc.robot.commands.superstructure.shooting.Shoot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.RotationArmSubsystem.Mode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    
    drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> leftJoystick.getY(), // Left up & down
        () -> leftJoystick.getX(), // Left side-to-side
        () -> rightJoystick.getX(), // Right side-to-side
        true
      )
    );

    rotationArmSubsystem.setDefaultCommand(new DefaultRotationArm(rotationArmSubsystem));

    //shooterSubsystem.setDefaultCommand(new Tracking(limelightSubsystem, turretSubsystem));

    configureButtonBindings();

  }

  // Where we put all of our button bindings
  private void configureButtonBindings() {

    new POVButton(gamepad, 0)
      .whenHeld(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(0.5))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(0.5))))
      .whenReleased(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(0.0))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(0.0))));

    new POVButton(gamepad, 180)
      .whenHeld(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(-0.5))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(-0.5))))
      .whenReleased(new InstantCommand(() -> telescopeArmSubsystem.setLeftPercent(0.0))
        .alongWith(new InstantCommand(() -> telescopeArmSubsystem.setRightPercent(0.0))));

    new POVButton(gamepad, 90)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.intakeArmPosition, Mode.Intaking)
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new POVButton(gamepad, 270)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.defaultArmPosition, Mode.Intaking)
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new JoystickButton(rightJoystick, 2)
      .whenPressed(() -> drive.resetIMU());

    new JoystickButton(gamepad, Button.kY.value)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.intakeArmPosition, 
        Mode.Intaking)
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)))
      .whenHeld(new Intake(indexingSubsystem, intakeSubsystem))
      .whenReleased(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.defaultArmPosition, 
        Mode.Intaking)
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new JoystickButton(gamepad, Button.kX.value)
      .whenPressed(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.intakeArmPosition, 
        Mode.Intaking)
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)))
      .whenHeld(new ReverseIndexing(indexingSubsystem, intakeSubsystem))
      .whenReleased(new UpdateRotationArm(rotationArmSubsystem, ClimberConstants.defaultArmPosition, 
        Mode.Intaking)
      .andThen(new HoldPositionRotationArms(rotationArmSubsystem)));

    new JoystickButton(gamepad, Button.kRightBumper.value)
      .whenHeld(new PrepareToShoot(shooterSubsystem, limelightSubsystem, 4000));
    
    new JoystickButton(gamepad, Button.kA.value)
      .whenHeld(new Shoot(indexingSubsystem));

    new JoystickButton(gamepad, Button.kBack.value)
      .whenPressed(new CalibrateTelescope(telescopeArmSubsystem)
      .alongWith(new HoodCalibrate(shooterSubsystem)));

    new JoystickButton(gamepad, Button.kStart.value)
      .whenPressed(new ClimbSequence(telescopeArmSubsystem, rotationArmSubsystem)
      .alongWith(new RunCommand(() -> turretSubsystem.runTurretPercent(0), turretSubsystem)));
    
  }

  // Prepares the robot for autonomous and sends the command we should use
  public Command getAutonomousCommand() {
   
    return null;

  }
}