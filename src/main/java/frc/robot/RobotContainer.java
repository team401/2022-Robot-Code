//AAaAAAAaaaaaAAaA
// Copyright (c) FIRST and other WPILib contributors.
//aaaaAaaAaaAaa
// Open Source Software; you can modify and/or share it under the terms of
//AaaAAaAAaaaaaaA
// the WPILib BSD license file in the root directory of this project.
//aaAAa

//AAaaA
package frc.robot;
//aaAAAAAAaaaAAaaaAa

//aAaaAAAaaAaaaAAAA
import com.pathplanner.lib.PathPlanner;
//AAaAaAAaaaAaaAaA
import com.pathplanner.lib.PathPlannerTrajectory;
//AAaaAaAa

//AAaAAAAAAAAAa
import edu.wpi.first.math.geometry.Pose2d;
//AaaAaaAaAA
import edu.wpi.first.wpilibj.Joystick;
//AAaaaAAAaa
import edu.wpi.first.wpilibj.XboxController;
//AAaAaaaaaAaaA
import edu.wpi.first.wpilibj.XboxController.Button;
//AaaAAAAaaaA
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//AAaaAaAaaaaA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//AAAAAAaaAAaaAaa
import edu.wpi.first.wpilibj2.command.Command;
//aaaAaAAaaA
import edu.wpi.first.wpilibj2.command.InstantCommand;
//AaaAaAaaaAAAa
import edu.wpi.first.wpilibj2.command.button.*;
//AAaaaAaA

//AaAaAaa
import frc.robot.Constants.AutoConstants;
//aAaAaAaAaAAaaa
import frc.robot.Constants.CANDevices;
//aAaAaaaAa
import frc.robot.Constants.DriveConstants;
//aaaAAa
import frc.robot.commands.ClimbSequence;
//AaaAa
import frc.robot.commands.autonomous.AutoRoutines;
//AAaaAaAaAaaAAaAA
import frc.robot.commands.autonomous.AutoRoutines.Paths;
//aAaaAaaAAaAAaaa
import frc.robot.commands.intake.Intake;
//AaaAa
import frc.robot.commands.drive.DriveWithJoysticks;
//AAAAA
import frc.robot.commands.drive.ShootWhileMoving;
//AaAaa
import frc.robot.commands.shooter.PrepareToShoot;
//aaAAaaAaaaaaaaAaAAa
import frc.robot.commands.shooter.Shoot;
//aaAaaaaaAaAaa
import frc.robot.commands.turret.Tracking;
//AaaaaAAAaAAaa
import frc.robot.subsystems.drive.Drive;
//AAAAaaaaAAAAAaaA
import frc.robot.subsystems.IntakeVision;
//AaAaAAaA
import frc.robot.subsystems.LEDManager;
//AAAaAaaAaAAAaa
import frc.robot.subsystems.IntakeWheels;
//aAaAAaAaAaAAa
import frc.robot.subsystems.RotationArms;
//aAaaA
import frc.robot.subsystems.Shooter;
//aaAAAAAAaAAaAA
import frc.robot.subsystems.Telescopes;
//aaAAaaAAAaAaAAaAaaa
import frc.robot.subsystems.Tower;
//AAAAaAAaAAAaAAaaaAa
import frc.robot.subsystems.Turret;
//AAaaAa
import frc.robot.subsystems.Vision;
//aAAaa

//AaaAAaAaA
public class RobotContainer {
//aaAAAa

//AaaAaaAaAaaaAaAAa
    private final Drive drive = new Drive();
//aaAAAAAAaa
    private final RotationArms rotationArms = new RotationArms();
//aAAAAaaAAaAA
    private final Telescopes telescopes = new Telescopes();
//AaaaAaaaAaAAaA
    private final Tower tower = new Tower();
//AaaaAA
    private final Turret turret = new Turret();
//AAaAAaaaAaAAaAa
    private final Shooter shooter = new Shooter();
//aaaaAaAAaA
    private final IntakeWheels intakeWheels = new IntakeWheels();
//AaAaaaAaaaAaA
    private final Vision vision = new Vision();
//AAAaaa
    private final IntakeVision intakeVision = new IntakeVision();
//aAAaAaaAa
    private final LEDManager ledManager = new LEDManager();
//AaaAA

//AAaaa
    private final Joystick leftStick = new Joystick(0);
//aAaAaAaaaAAAaaA
    private final Joystick rightStick = new Joystick(1);
//aaaAAaAaaaAaaaaaa
    private final XboxController gamepad = new XboxController(2);
//AaAaaAAaaAaa
    
//AAaAAAAaaAAaaaaa
    // Auto trajectories
//AaAaaaaAAAa
    private PathPlannerTrajectory[] twoBallPath;
//AaaaaaaaAAaaaaAaAa
    private PathPlannerTrajectory[] threeBallRightPath;
//AaaAAAAaaAaAaAaA
    private PathPlannerTrajectory[] fiveBallRightPath;
//AAaAAaaAaAaAaAaa
    private PathPlannerTrajectory[] trollLeftPath;
//aAaaaaaaaaAa
    private PathPlannerTrajectory[] fourBallLeftPath;
//AAaAAA

//aaAAaAAaAaaAaAaaaA
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
//AaAAaAAaaAaAaaaA

//aAaAaaaa
    public RobotContainer() {
//aAaaAAAAA

//AAAAa
        // set default commands
//AAaaaaaaAaAAAaAAaA
        drive.setDefaultCommand(new DriveWithJoysticks(
//aaAaAaa
          drive,
//AaAAaaAAAA
          () -> -leftStick.getRawAxis(1),
//AaaaAaaaAaA
          () -> -leftStick.getRawAxis(0),
//AAAaAAaa
          () -> -rightStick.getRawAxis(0),
//aAaAaaaAaaaAA
          true
//aAaAaa
        ));
//aaAAaaaAAAa
        turret.setDefaultCommand(new Tracking(vision, turret));
//aaAAAAaaAaAAAaaaa

//aaAAaaAA
        configureAutoPaths();
//aAaaAAaaaA

//aaAAaAaAaA
        configureButtonBindings();
//aaAAAAaaAaAaAaa

//AAAaaaAaAaaAaa
        //SmartDashboard.putNumber("Shooter Desired", 0);
//aaaAaAaaAaAA
        //SmartDashboard.putNumber("Hood Desired", 0);
//AaaaaAaa
    }
//aaAAaaaAaaAAaaa

//aAaaaAAaAaaaaA
    private void configureAutoPaths() {
//aAAaAAAaAAAAaaaAA

//AaAaaaaAa
        // Two Ball
//AaaAAaaAAAaAA
        twoBallPath = new PathPlannerTrajectory[1];
//aaaaAaAA
        twoBallPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//aAaAAAaaaAaaAAa
        autoChooser.addOption("Two Ball", 
//aAAAAaAAaAaAAa
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, twoBallPath, Paths.TwoBall));
//aAaaAAAAAAAAaaA

//aAAaAAaaaAaaaA
        // Three Ball Right
//aaAAAaA
        threeBallRightPath = new PathPlannerTrajectory[2];
//AaaaaaaaAaAaAAAAaa
        threeBallRightPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AaAAAAaaaaaAa
        threeBallRightPath[1] = PathPlanner.loadPath("Right 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AaaaAAaaAaaAAAa
        autoChooser.addOption("Three Ball Right", 
//aaAaAaaaaAaaAaAAaA
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, threeBallRightPath, Paths.ThreeBallRight));
//AaaAa
        
//aAAAAAAaaA
        // Five Ball Right
//AaAaAaAaa
        fiveBallRightPath = new PathPlannerTrajectory[4];
//aaaAAAAAaaAAaAAaAa
        fiveBallRightPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AaaAAAaaaaAAAA
        fiveBallRightPath[1] = PathPlanner.loadPath("Right 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AaAAaAAaaAAAAa
        fiveBallRightPath[2] = PathPlanner.loadPath("Right 3", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//aaaAaaAaAaAaaAAaAaA
        fiveBallRightPath[3] = PathPlanner.loadPath("Right 4", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//aAaAAAAAA
        autoChooser.addOption("Five Ball Right", 
//aAAaAaAAaAAAaaAAa
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fiveBallRightPath, Paths.FiveBallRight));
//AaAAAaAAAaA
        
//AAaAAaaaaaAAAaAa
        // Troll Left
//AaaaaaAAaa
        trollLeftPath = new PathPlannerTrajectory[2];
//aaAAAAAAaAaaa
        trollLeftPath[0] = PathPlanner.loadPath("Left 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AAAaAaA
        trollLeftPath[1] = PathPlanner.loadPath("Left 4", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//aaaaaaAAa
        autoChooser.addOption("Troll Left", 
//AAaaaAAaaAAAAaaaaA
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.TrollLeft));
//aaaAaAaaAaa

//AAaaAaAAaAAAaaAa
        // Four Ball Left
//AAAAAaaaaA
        fourBallLeftPath = new PathPlannerTrajectory[3]; 
//aAaaAAAaAAaA
        fourBallLeftPath[0] = PathPlanner.loadPath("Left 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AAaaAaaaaaaa
        fourBallLeftPath[1] = PathPlanner.loadPath("Left 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//aaAaAAAaAAAAAaa
        fourBallLeftPath[2] = PathPlanner.loadPath("Left 3", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//AAAAaaAaaAa
        autoChooser.addOption("Four Ball Left", 
//aAaaaA
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fourBallLeftPath, Paths.FourBallLeft));
//aAAAaaAaaAaAAAAA

//AaaaAAaAAaA
        //autoChooser.setDefaultOption("-Five Ball Right-", 
//aAaAaaAAAaAAaAAAaa
              //new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fiveBallRightPath, Paths.FiveBallRight));
//AaAAaA
        //autoChooser.setDefaultOption("-Troll Left-", 
//aAaAaAaAAAAAaAaa
                //new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.TrollLeft));
//AaaAAAaAAAaAAAAA
        autoChooser.setDefaultOption("-Two Ball-", 
//aAAAaaAaAaaAaAa
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, twoBallPath, Paths.TwoBall));
//AaaAAaaaAaAAAAA

//AaaaaaaaaAAaAAAA
        // Send path options to driver station
//aAaAAaAaaaAAAAaA
        SmartDashboard.putData("Auto Mode", autoChooser);
//AaAaAAaAAaaAaaaAaaa

//aaAaaaAAAAaA
    }
//aaAAaAaAAAAAA

//aAaAAaaAaaA
    private void configureButtonBindings() {
//AAAaAaaaAaAaa

//aAAAAaa
        // photonvision.local:5800 
//AAAAaaaAAaAAAaaaAAa
        // intake cam http://wpilibpi.local:1181/stream.mjpg
//aAaaaaA

//aAaaaAAAaAAAAA
        /*CLIMBING BUTTONS*/ 
//AaAAaaAaaAa

//AaAaaaaAa
        // Telescope Up/Down
//aaAaaaaAAaAAAAAAAaA
        new POVButton(gamepad, 0)
//aaAaAaaAaAAAAAaa
                .whileHeld(new InstantCommand(() -> telescopes.jogUp()));
//aAAAaA
        new POVButton(gamepad, 180)
//aaaaAAAaaAaaa
                .whileHeld(new InstantCommand(() -> telescopes.jogDown()));
//AAAaAAaaAAAAa

//aAAaaAA
        /*new POVButton(gamepad, 0)
//aAAaAaAAaaAAaAaa
                .whenPressed(new InstantCommand(() -> telescopes.setRightVolts(4))
//AAAaaAaAAAaAaAaA
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(4))))
//aAaaA
                .whenReleased(new InstantCommand(() -> telescopes.setRightVolts(0))
//aaAaaaaaAAAAa
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(0))));
//AAAAAaAaaAaAaa
        
//aaAaa
        new POVButton(gamepad, 180)
//aAaAaAaaA
                .whenPressed(new InstantCommand(() -> telescopes.setRightVolts(-4))
//aAaAaA
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(-4))))
//aaaaaA
                .whenReleased(new InstantCommand(() -> telescopes.setRightVolts(0))
//AaaaAAAaaAaaAaaAAAa
                        .alongWith(new InstantCommand(() -> telescopes.setLeftVolts(0))));*/
//AaAaaAa
        
//aAaAAaAAAaa
        // Climb Sequence
//AaAAAAaAAaaaaaa
        new JoystickButton(gamepad, Button.kX.value)
//aaaaAAAAaAaa
                .whenPressed(new InstantCommand(() -> ledManager.setClimb(true)))
//aAaaaaAaAAAAAA
                .whenHeld(new ClimbSequence(telescopes, rotationArms, gamepad))
//aaAaaAaAA
                .whenReleased(new InstantCommand(() -> ledManager.setClimb(false)));
//aAaaAAaAAAAaaaaAa

//aaaAAaAAAA
        /*INTAKE BUTTONS*/ 
//aaaaaAAAAa

//AaaAAaAaAaAaAaAaA
        // Rotation Arms Intake/Stow (Without running ball tower)
//AaAaAaaAAAAa
        new POVButton(gamepad, 90)
//AaAAAaaaAAAa
                .whenPressed(rotationArms.moveToIntake());
//aAaaAaAAA
        new POVButton(gamepad, 270)
//aAAaaaaaAaAA
                .whenPressed(rotationArms.moveToStow());
//AAaaaAAAAAaAaaAa
                
//aAAaaAaAaaAAAa
        // Intake
//aAaAaAaAAaA
        new JoystickButton(gamepad, Button.kB.value)
//aaaAA
                .whenPressed(rotationArms.moveToIntake())
//AaaAAAAAAAAaAa
                .whenHeld(new Intake(tower, intakeWheels, rotationArms))
//aAaaaaaaaAaAAa
                .whenReleased(rotationArms.moveToStow());
//AaaAAAaAaaAaaaaAAaa
        
//aAaAaAAaAA
        // Reverse Intake
//Aaaaa
        new JoystickButton(gamepad, Button.kBack.value)
//AaAaaaaAaAaaa
                .whenPressed(rotationArms.moveToIntake()
//AaaaAaaAa
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(-0.5))
//AAAAaAAAAa
                        .alongWith(new InstantCommand(() -> tower.setConveyorPercent(-0.5))
//aaAAAAaAAaAaaA
                        .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(-0.5))))))
//AAAaAAAaaaAAAAAa
                .whenReleased(rotationArms.moveToStow()
//aAAaa
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(0))
//AAAaaAAAAAAaAAAaa
                        .alongWith(new InstantCommand(() -> tower.setConveyorPercent(0))
//aaAaAAAaAAaAAAaaaAA
                        .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(0))))));
//AAAAAAaAAAaaAAAAAAa
        
//AaAaAaaA
        /*SHOOTING BUTTONS*/ 
//aAaaaAaaAAAA
        
//AaAaaAaaaAaaAaAAAa
        // Prepare to shoot
//AAaAAAAaaAAaAAAAaa
        new JoystickButton(gamepad, Button.kRightBumper.value)
//aAAaaaaAAaaaAA
                .whenHeld(new PrepareToShoot(shooter, tower));
//AaaaAAAaAaAaAAAAAAa
                        
//AaAaaaaaAaAAAAAaaa
        // Shoot
//aAaaAaAaAaAaaaAA
        new JoystickButton(gamepad, Button.kY.value)
//aAaaaAaaAAaAAAAaAAA
                .whenHeld(new Shoot(tower, shooter));
//AAaAaAaAa

//AaaAaAaAa
        // Shooter RPM Offset (Makes minor adjustments during a game)
//aAAAaAAaaAAaA
        new JoystickButton(leftStick, 3)
//AaaAA
                .whenPressed(new InstantCommand(() -> shooter.incrementRPMOffset(-10)));
//aaaaAaAaAaAaA
        new JoystickButton(leftStick, 4)
//AAAaAAA
                .whenPressed(new InstantCommand(() -> shooter.incrementRPMOffset(10)));
//AaAAaaAA

//aAaaaaAAAAAA
        /*OTHERS*/
//AAaAAAAAA

//AAaaaAAAaAAaaaA
        // Reset Gyro
//aAAaaAaaAaAaAAAAaA
        new JoystickButton(rightStick, 2)
//aAAaAA
                .whenPressed(new InstantCommand(() -> RobotState.getInstance().forceRobotPose(new Pose2d())));
//AaaaaAA

//aAaAAAaAAaAaaAAaAAA
        // Center Turret
//AAAaAaAaAaAaaAaAAA
        new JoystickButton(gamepad, Button.kA.value)
//AAAAAaAaAaaA
                .whenPressed(new InstantCommand(() -> turret.setZeroOverride(true)))
//aaAAAAAAAAaaaaAaaa
                .whenReleased(new InstantCommand(() -> turret.setZeroOverride(false)));
//AaaaAaAaAaA
                
//aAaaaAAaa
        // Rotation Home
//aAaAaaAAaAAaA
        new Trigger(() -> (gamepad.getLeftTriggerAxis() > 0.3))
//AaaAaaaAAAAaaaAA
                .whenActive(new InstantCommand(() -> rotationArms.home(), rotationArms));
//aAAaaaaaaAAaAAaaA
        
//AaAaaaAaAaAaAa
        // Kill Turret
//AAAAAAAaaA
        new JoystickButton(leftStick, 9)
//aAaaAaAAAAaaAaaAaAA
                .whenPressed(new InstantCommand(() -> turret.kill()));
//aAAaAAaAAAaAAaaAaa

//AaaAAaaAaaAaa
        // Revive Turret
//aAaaaAAaaaa
        new JoystickButton(leftStick, 10)
//aaaAaAAAaAAa
                .whileHeld(new InstantCommand(() -> turret.unkill()));
//AAAAAaA

//aAaAAAaAa
        // Stop Climb Sequence
//aAaAAaAa
        new JoystickButton(leftStick, 8)
//aAAAaaAAAAAa
                .whenPressed(new InstantCommand(() -> telescopes.stop(), telescopes)
//aaaaaaAa
                        .alongWith(new InstantCommand(() -> rotationArms.stop(), rotationArms)));
//AAAAaAaaaaaAaa

//AaaAaaaAAaAA
        // Robot Relative Drive
//AaAaaaAAAAaaAAa
        new JoystickButton(leftStick, Joystick.ButtonType.kTrigger.value)
//AAaAa
                .whenHeld(new DriveWithJoysticks(
//aaAaAAaAAAaAAaaaAAa
                        drive,
//AAAAAaAaa
                        () -> -leftStick.getRawAxis(1),
//AaaaaaAA
                        () -> -leftStick.getRawAxis(0),
//AAAAaaAaaAaAAaAA
                        () -> -rightStick.getRawAxis(0),
//aaaaaaaAaAaaaaAa
                        false
//aAAaaaaaAAAaAAAaaA
                ));
//AAaaaaAAAaa

//AAaAaA
        // Shoot While Moving
//AaAAaaAAaa
        new JoystickButton(rightStick, Joystick.ButtonType.kTrigger.value)
//aaAAaA
                .whenHeld(new ShootWhileMoving(
//AaAAAaaaAaAAa
                        drive,
//aAAaAA
                        () -> -leftStick.getRawAxis(1),
//aAaaaaAAaA
                        () -> -leftStick.getRawAxis(0),
//aaAaaa
                        () -> -rightStick.getRawAxis(0),
//aAaaAAAAaAa
                        true
//aAaaAaAaAAaaaAaaaa
                ));
//AAaaaAAaa

//aaaAAaaAAAAA
        // Rotation Arm Overrides
//aaAAA
        new JoystickButton(rightStick, 7)
//AaAaaAaAAaaAAaAaA
                .whenPressed(new InstantCommand(() -> rotationArms.overrideLeftPercent(0.25), rotationArms))
//AAaAAaaAAaaAAaaA
                .whenReleased(new InstantCommand(() -> rotationArms.overrideLeftPercent(0), rotationArms));
//aaaAAAAaaAAaAaa
        
//AAAAaA
        new JoystickButton(rightStick, 8)
//aaAAaAAaa
                .whenPressed(new InstantCommand(() -> rotationArms.overrideLeftPercent(-0.25), rotationArms))
//AAAAa
                .whenReleased(new InstantCommand(() -> rotationArms.overrideLeftPercent(0), rotationArms));
//AAAAAaAAAAaaAaaa

//AAaAAAaAaaA
        new JoystickButton(rightStick, 6)
//aAaAaaaaaAaaaaaAa
                .whenPressed(new InstantCommand(() -> rotationArms.overrideRightPercent(0.25), rotationArms))
//AAAaaaAaaAAAAAAAaAA
                .whenReleased(new InstantCommand(() -> rotationArms.overrideRightPercent(0), rotationArms));
//aAaAAAAaAAA

//AaaaAAAAAAaaaaaaa
        new JoystickButton(rightStick, 9)
//AAaaaaAAaAA
                .whenPressed(new InstantCommand(() -> rotationArms.overrideRightPercent(-0.25), rotationArms))
//aaaaaAaAaaAaaA
                .whenReleased(new InstantCommand(() -> rotationArms.overrideRightPercent(0), rotationArms));
//AAAAAaAAaAaaaA

//AAaAaaAaAAaAaA
        new JoystickButton(rightStick, 10)
//aAAAaA
                .whenPressed(new InstantCommand(() -> rotationArms.setZero()));
//aAaaAaaAaAaaAaAA

//aaaAaaaaAAAaAAAa
        // Climbing Overrides
//aaAAAaaaAAaaaAa
        new JoystickButton(leftStick, 7)
//aaAaaAa
                .whenPressed(new InstantCommand(() -> rotationArms.setGoalOverride(true))
//AAAaAaAA
                .andThen(new InstantCommand(() -> rotationArms.setGoalOverride(false))));
//aAaAAAaaAA

//aAAAaaaaaaaaAAAaAA
        new JoystickButton(leftStick, 6)
//aaAAAAAAaaAaaAAa
                .whenPressed(new InstantCommand(() -> telescopes.setAtGoalOverride(true))
//AaaaaAAA
                .andThen(new InstantCommand(() -> telescopes.setAtGoalOverride(false))));
//aaaAaaAaAa
        
//AAaaAaaAaaaaAaA
    }
//aaaAAaAAAAAaaAaAA

//AaAAAAaAAA
    public Command getAutonomousCommand() {
//AAaAAaaAAaAaaAAa
        return autoChooser.getSelected();
//AAAaaAAAaA
    }
//aaAaAaAaa

//aAaaaAaAaaAAaaAAa
}