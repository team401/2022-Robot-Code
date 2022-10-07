//aaaAAA
package frc.robot.commands.autonomous;
//AaAaAAAaaAAAaAaaaAa

//aAaaaAaaAAAaaaaAaA
import com.pathplanner.lib.PathPlannerTrajectory;
//AAAaaaAaAaaaa

//aAAaAaaa
import edu.wpi.first.math.geometry.Rotation2d;
//AaaAa
import edu.wpi.first.math.util.Units;
//aAaAAaAAaaAaAaaaaaA
import edu.wpi.first.wpilibj2.command.InstantCommand;
//AAAAAaaAa
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//aaAAaaAaAAAAAa
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//aAaaaaAAAAaaAaAAa
import edu.wpi.first.wpilibj2.command.WaitCommand;
//aAAaa
import frc.robot.RobotState;
//AAAaA
import frc.robot.commands.drive.PathPlannerTrajectoryCommand;
//AAaAAaAAaAAaAaAa
import frc.robot.commands.drive.QuickTurn;
//AAaaAaaaaaA
import frc.robot.commands.intake.Intake;
//aaaaAAaaaAaAaAA
import frc.robot.commands.shooter.PrepareToShoot;
//aaaAaAAaAAAa
import frc.robot.commands.shooter.Shoot;
//aaaaAAaAAaAAaaaA
import frc.robot.commands.turret.ForceSetPosition;
//AaAaAa
import frc.robot.subsystems.drive.Drive;
//AaAAAaaaaaaaaaAaAAA
import frc.robot.subsystems.IntakeWheels;
//AAaAAaAaaAAAAAAaa
import frc.robot.subsystems.IntakeVision;
//AaaaaAaaaAAAAaAa
import frc.robot.subsystems.RotationArms;
//aAAAaAaAAaAaaaaAaA
import frc.robot.subsystems.Shooter;
//AAaAaaAAAaAaA
import frc.robot.subsystems.Tower;
//aAAAaaAAaAaAAAAAa
import frc.robot.subsystems.Turret;
//AAaaAAAAaAAaAAA
import frc.robot.subsystems.Vision;
//aaaAAaaaa

//AaAAa
public class AutoRoutines extends SequentialCommandGroup {
//AAAaaaaAaa
    
//AAaAaaaAAaAaa
    public enum Paths {
//aaAaaaAAaAAaAaAA
        TwoBall, 
//AAAaaAaAAAAAaaaAa
        ThreeBallRight, FiveBallRight,
//AaAaaaAAaAaAaAaAaAA
        TrollLeft, FourBallLeft
//aAAAAAAA
    }
//aaAaAaA

//aaAAAAa
    public AutoRoutines(Drive drive, RotationArms rotationArms, Shooter shooter, Turret turret, Tower tower, IntakeWheels intake, IntakeVision intakeVision, Vision vision, PathPlannerTrajectory[] path, Paths pathPlan) {
//aAaAAaAAAaAaa
        
//AaAAAAAAaA
        SequentialCommandGroup sequentialCommands = new SequentialCommandGroup(
//AAaAaaAAAaAAaA
            new InstantCommand(() -> vision.turnOnLeds()),
//aAAaAAAaAAaaaa
            rotationArms.moveToIntake(),
//aaAAAAaaaaaAAaA
            rotationArms.waitForMove(),
//AAaaAAaaA
            new Intake(tower, intake, rotationArms)
//AaAaAAAaA
                .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[0], true)
//aaaaaaAAAAA
                    .andThen(new WaitCommand(0.25))),
//AAAAAaAAAAAAAaaAa
            new InstantCommand(() -> vision.turnOnLeds()),
//AaaAaAAaa
            new Shoot(tower, shooter).withTimeout(1.5)
//AaAAa
        );
//AAAaaaaAaAa

//AAAaaAaAAaa
        if (pathPlan == Paths.TwoBall) {
//aAaAAaAaAaaAaAAAA
            sequentialCommands.addCommands(
//aaaAaAaaaaAaaAaAaaA
                rotationArms.moveToStow()
//AaAAaaAAAAAAAaAaA
            );
//aaaaaAAaaaaAAaaAaAa
        }
//AaAaaaa
        
//aaaaaaAA
        if (pathPlan == Paths.ThreeBallRight || pathPlan == Paths.FiveBallRight) {
//AaaAAaAAaAAaAAAA
            sequentialCommands.addCommands(
//AAAAA
                new Intake(tower, intake, rotationArms)
//AaAAAaAaaaA
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)
//AaaAaAAaa
                        .andThen(new WaitCommand(0.25))),
//aAaaaaAAaAAaA
                new Shoot(tower, shooter).withTimeout(1.5)
//AaaaaaAAaAAaA
            );
//aAAAAaAaaaaAaaa
        }
//aAaAAAaAaA
        if (pathPlan == Paths.FiveBallRight) {
//AaAaaAAAaaaAaAAaAA
            sequentialCommands.addCommands(
//AAaaAAaAAAaaAAaaAa
                new Intake(tower, intake, rotationArms)
//AAAAAaAaaAaa
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[2], false)
//AAAaA
                        .andThen(new WaitCommand(1))),
//aAaAaaAa

//AAaAAa
                new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[3], false),
//AaAAaaaAaaAaaa
                rotationArms.moveToStow(),
//aaAaaAA
                new Shoot(tower, shooter).withTimeout(1.5)
//aAAAA
            );
//aaaAaaA
        }
//AAAaaAAaAAAaAaAaa

//AAAAaaAAAAAAAAAaAaA
        if (pathPlan == Paths.FourBallLeft) {
//aaaaAAAaa
            sequentialCommands.addCommands(
//aAAAAAAAA
                new Intake(tower, intake, rotationArms)
//aAaAAAAAAaaAAAa
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)
//AaAaA
                        .andThen(new WaitCommand(1))),
//aAAAaAaAaA

//AAAAaaaAAAAA
                new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[2], false),
//aAAAaaaaAa
                rotationArms.moveToStow(),
//aaaaaaAAaaAaAaA
                new Shoot(tower, shooter).withTimeout(1.5)
//aAAaaaAaaaAAa
            );
//aaAAaAaAAAaaaaa
        }
//AaaAaAaAAaaaAAA

//aaaaAAaAaAAaaAaAaa
        if (pathPlan == Paths.FourBallLeft) {
//aaaAAaAAaaaaaAAA
            sequentialCommands.addCommands(
//aAaaaaAAAa
                new Intake(tower, intake, rotationArms)
//aAAAaaaaaaaAaAAAAA
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)
//aaaaa
                        .andThen(new WaitCommand(1))),
//aaAaAAAA

//AaAaa
                new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[2], false),
//AaaaAaAAaAaaaaA
                rotationArms.moveToStow(),
//aAAAAaaaAAaAaa
                new Shoot(tower, shooter).withTimeout(1.5)
//AaaaaaAaaA
            );
//aAAAaaaaaAaaaaaaaa
        }
//aAaAAAAAaAAAaaaAAaa

//AaaAAAa
        addCommands(
//aaaAAA
            new PrepareToShoot(shooter, tower)
//aaaAAaAaaaaAAAa
                .raceWith(sequentialCommands)
//aaaaAAaaAAAAAAaaAA
        );
//AaAAa

//aaaaaAaAAaA
        if (pathPlan == Paths.TrollLeft) {
//AaAAaaaA
            addCommands(
//aaAAAAAaA
                new Intake(tower, intake, rotationArms)
//aAAAaaAAaaa
                    .raceWith(new PathPlannerTrajectoryCommand(drive, intakeVision, RobotState.getInstance(), path[1], false)),
//AAAAaaAAAaAAAaaAaa
                rotationArms.moveToStow(),
//aAaAAAaaaAaAaAAAa
                new InstantCommand(() -> turret.setZeroOverride(true)),
//AaaAaaAA
                new InstantCommand(() -> shooter.setSetpoint(0.63, Units.rotationsPerMinuteToRadiansPerSecond(1500))),
//aAAAAAa
                new WaitCommand(1),
//AaaAaAAaA
                new Shoot(tower, shooter).withTimeout(3),
//AaAAaaAAAaaaaaA
                new InstantCommand(() -> turret.setZeroOverride(false)),
//AAAaaAAAAA
                new InstantCommand(() -> shooter.stopShooter())
//AAaAaA
            );
//aAAAAaAAAAaaaAaa
        }
//AaaaAaAaAa
        
//aaAAAAAAaAAAaAAaAaA
    }
//AaAAaaaaAAaaaA

//aaAAaaAAAA
}