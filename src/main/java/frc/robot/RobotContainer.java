// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.commands.autonomous.AutoRoutines.Paths;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.MeasureWheelRadius;
import frc.robot.commands.drive.ShootWhileMoving;
import frc.robot.commands.shooter.DemoShoot;
import frc.robot.commands.shooter.PrepareToShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.turret.DemoStick;
import frc.robot.commands.turret.Tracking;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.RotationArms;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telescopes;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    private final Drive drive = new Drive();
    private final RotationArms rotationArms = new RotationArms();
    private final Telescopes telescopes = new Telescopes();
    private final Tower tower = new Tower();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final IntakeWheels intakeWheels = new IntakeWheels();
    private final Vision vision = new Vision();
    private final IntakeVision intakeVision = new IntakeVision();
    private final LEDManager ledManager = new LEDManager();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);
    
    // Auto trajectories
    private PathPlannerTrajectory[] twoBallPath;
    private PathPlannerTrajectory[] threeBallRightPath;
    private PathPlannerTrajectory[] fiveBallRightPath;
    private PathPlannerTrajectory[] trollLeftPath;
    private PathPlannerTrajectory[] fourBallLeftPath;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public RobotContainer() {

        // set default commands
        drive.setDefaultCommand(new DriveWithJoysticks(
          drive,
          () -> -leftStick.getRawAxis(1),
          () -> -leftStick.getRawAxis(0),
          () -> -rightStick.getRawAxis(0),
          true
        ));
        //turret.setDefaultCommand(new Tracking(vision, turret));
        turret.setDefaultCommand(new DemoStick(vision, turret, () -> gamepad.getRawAxis(0), () -> gamepad.getRawAxis(1)));

        shooter.setDefaultCommand(new DemoShoot(shooter, tower, () -> gamepad.getRightTriggerAxis()));

        configureAutoPaths();

        configureButtonBindings();

    }

    private void configureAutoPaths() {

        // Two Ball
        twoBallPath = new PathPlannerTrajectory[1];
        twoBallPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Two Ball", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, twoBallPath, Paths.TwoBall));

        // Three Ball Right
        threeBallRightPath = new PathPlannerTrajectory[2];
        threeBallRightPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        threeBallRightPath[1] = PathPlanner.loadPath("Right 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Three Ball Right", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, threeBallRightPath, Paths.ThreeBallRight));
        
        // Five Ball Right
        fiveBallRightPath = new PathPlannerTrajectory[4];
        fiveBallRightPath[0] = PathPlanner.loadPath("Right 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightPath[1] = PathPlanner.loadPath("Right 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightPath[2] = PathPlanner.loadPath("Right 3", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightPath[3] = PathPlanner.loadPath("Right 4", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Five Ball Right", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fiveBallRightPath, Paths.FiveBallRight));
        
        // Troll Left
        trollLeftPath = new PathPlannerTrajectory[2];
        trollLeftPath[0] = PathPlanner.loadPath("Left 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        trollLeftPath[1] = PathPlanner.loadPath("Left 4", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Troll Left", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.TrollLeft));

        autoChooser.addOption("Nothing", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.Nothing));

        // Four Ball Left
        /*fourBallLeftPath = new PathPlannerTrajectory[3]; 
        fourBallLeftPath[0] = PathPlanner.loadPath("Left 1", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeftPath[1] = PathPlanner.loadPath("Left 2", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeftPath[2] = PathPlanner.loadPath("Left 3", AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        autoChooser.addOption("Four Ball Left", 
                new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fourBallLeftPath, Paths.FourBallLeft));*/

        autoChooser.setDefaultOption("-Five Ball Right-", 
              new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, fiveBallRightPath, Paths.FiveBallRight));
        //autoChooser.setDefaultOption("-Troll Left-", 
                //new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, trollLeftPath, Paths.TrollLeft));
        //autoChooser.setDefaultOption("-Two Ball-", 
                //new AutoRoutines(drive, rotationArms, shooter, turret, tower, intakeWheels, intakeVision, vision, twoBallPath, Paths.TwoBall));

        // Send path options to driver station
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    private void configureButtonBindings() {

        // intake cam http://10.4.1.217:1181
        // intake cam video http://10.4.1.217:1181/stream.mjpg

        /*CLIMBING BUTTONS*/ 

        // Telescope Up/Down
        new POVButton(gamepad, 0)
                .whileHeld(new InstantCommand(() -> telescopes.jogUp()));
        new POVButton(gamepad, 180)
                .whileHeld(new InstantCommand(() -> telescopes.jogDown()));
        
        // Climb Sequence
        new JoystickButton(gamepad, Button.kX.value)                    
                .whenPressed(new InstantCommand(() -> ledManager.setClimb(true)))
                .whenHeld(new ClimbSequence(telescopes, rotationArms, turret, vision, gamepad))
                .whenReleased(new InstantCommand(() -> ledManager.setClimb(false)));

        /*INTAKE BUTTONS*/ 

        // Rotation Arms Intake/Stow (Without running ball tower)
        new POVButton(gamepad, 90)
                .whenPressed(rotationArms.moveToIntake());
        new POVButton(gamepad, 270)
                .whenPressed(rotationArms.moveToStow());
                
        // Intake
        new JoystickButton(gamepad, Button.kB.value)
                .whenPressed(rotationArms.moveToIntake())
                .whenHeld(new Intake(tower, intakeWheels, rotationArms))
                .whenReleased(rotationArms.moveToStow());

        // Reverse Intake
        new JoystickButton(gamepad, Button.kBack.value)
                .whenPressed(rotationArms.moveToIntake()
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(-0.5))
                        .alongWith(new InstantCommand(() -> tower.setConveyorPercent(-0.5))
                        .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(-0.5))))))
                .whenReleased(rotationArms.moveToStow()
                        .alongWith(new InstantCommand(() -> intakeWheels.setPercent(0))
                        .alongWith(new InstantCommand(() -> tower.setConveyorPercent(0))
                        .alongWith(new InstantCommand(() -> tower.setIndexWheelsPercent(0))))));
        
        /*SHOOTING BUTTONS*/       
        // Shoot
        new JoystickButton(gamepad, Button.kY.value)
                .whenHeld(new Shoot(tower, shooter));

        /*OTHERS*/

        // Reset Gyro
        new JoystickButton(rightStick, 2)
                .whenPressed(new InstantCommand(() -> RobotState.getInstance().forceRobotPose(new Pose2d())));
                
        // Rotation Home
        /*new Trigger(() -> (gamepad.getLeftTriggerAxis() > 0.3))
                .whenActive(new InstantCommand(() -> rotationArms.home(), rotationArms));

        // Telescope Home
        new JoystickButton(gamepad, Button.kBack.value)
            .whenActive(new InstantCommand(() -> telescopes.home(), telescopes));*/
        
        // Kill Turret
        new JoystickButton(leftStick, 9)
                .whenPressed(new InstantCommand(() -> turret.kill()));

        // Revive Turret
        new JoystickButton(leftStick, 10)
                .whileHeld(new InstantCommand(() -> turret.unkill()));
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}