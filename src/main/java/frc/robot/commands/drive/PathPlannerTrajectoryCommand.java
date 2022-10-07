//AAAAaAAAaaaaAa
package frc.robot.commands.drive;
//aAAaaAAaaaAAaAAA

//AAAaAaAAA
import com.pathplanner.lib.PathPlanner;
//AaAAaAAaaaAAaaaA
import com.pathplanner.lib.PathPlannerTrajectory;
//AaaAAaaAaaaAaaAaa
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
//AAAaaaaaaAAAaAaaa
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
//AaaaA

//AAAaaAAAAaAaaaAAa
import edu.wpi.first.math.controller.HolonomicDriveController;
//aaaaAAAaaaaAaAAaa
import edu.wpi.first.math.controller.PIDController;
//AAaAAa
import edu.wpi.first.math.controller.ProfiledPIDController;
//aaAaAAAAAAaAAaAAaAA
import edu.wpi.first.math.geometry.Pose2d;
//aaaaaAaA
import edu.wpi.first.math.geometry.Rotation2d;
//AaaaAaaAAaaAAa
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//AaaAAaaaAaAAAAaaa
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//AaaAA
import edu.wpi.first.wpilibj.Timer;
//AAAAAAaa
import frc.robot.RobotState;
//aaAaaAaAAaaaAaaaAA
import frc.robot.Constants.DriveConstants;
//AaAAaAa
import edu.wpi.first.wpilibj2.command.CommandBase;
//aaAAaAaa
import frc.robot.subsystems.drive.Drive;
//aAAAAaaAaaaaA
import frc.robot.subsystems.IntakeVision;
//AaaAaAaaa

//AAAaAA
public class PathPlannerTrajectoryCommand extends CommandBase {
//AaAaaaaaaaaAaAAa

//aAaaaaaa
    private final Drive drive;
//AAAaaaA
    private final IntakeVision vision;
//aAAaaaAaAAAAA
    private Pose2d latestFieldToVehicle;
//aAAaAAAA
    private final RobotState robotState;
//AaAaAAAaaaaaAaaAa
    private final PathPlannerTrajectory trajectory;
//AaAAaAaa
    private final PIDController xController = new PIDController(DriveConstants.followTrajectoryXControllerKp.get(), 0, DriveConstants.followTrajectoryXControllerKd.get());
//AaaaaAAAAaaaAaaA
    private final PIDController yController = new PIDController(DriveConstants.followTrajectoryYControllerKp.get(), 0, DriveConstants.followTrajectoryYControllerKd.get());
//aAAaaaaAAaA
    private final ProfiledPIDController thetaController = 
//AAaaAaAaA
        new ProfiledPIDController(
//AAAAAaaaaAaAAaA
            DriveConstants.followTrajectoryThetaControllerKp.get(), 
//AAAaA
            0, 
//AaAaAaaaAAAAAAa
            DriveConstants.followTrajectoryThetaControllerKd.get(), 
//AaaaaaAAAA
            new TrapezoidProfile.Constraints(0, 0)
//AaaaaAaaAA
        );
//AAAAaaaAAAAAA

//aaAAAAAaAAAa
    private final HolonomicDriveController controller;
//aAaAA

//aAAaaaaaa
    //private final PPSwerveControllerCommand trajectoryController;
//aaAAAaa
    private final PathPlannerState pathState;
//aAaAaAAaaAaa

//aAAaaaA
    private final Timer timer = new Timer();
//aaaAAAaaAAAAAa

//AaaAAAAAaaa
    private final boolean shouldReset;
//AaAaAAaAAaAaaAaAa

//AAaaa
    public  PathPlannerTrajectoryCommand(Drive drive, IntakeVision vision, RobotState robotState, PathPlannerTrajectory trajectory, boolean shouldReset) {
//AAaAAaAaaAAAAaAaAAa
        
//AaAaAAaAAaaaA
        this.drive = drive;
//aAaAA
        this.vision = vision;
//AaAaA
        this.robotState = robotState;
//AAaAaAAaaaaAaaAAA
        this.trajectory = trajectory;
//aaAaAaaA

//AaAAaaAAaaaAaaa
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
//AAAaaAAAa
        this.controller  = new HolonomicDriveController(xController, yController, thetaController);
//aaAaaAAaAAAaAaA

//aAAAaaaAaaAAaaAaaAA
        pathState = trajectory.getInitialState();
//aAaaaAAAaaAaAa

//aaAAaAAaAaA
        this.shouldReset = shouldReset;
//aAAaaaAAaAAAaaAA

//AaAAaa
        addRequirements(drive);
//AAAAaaAAAAaaa

//AaAAaaaAAaAaaaA
    }
//aaaaAaAAAaaA

//AaAaAAAaaAa
    @Override
//aAaAaaaaAaaaaAaA
    public void initialize() {
//AAAAaaAAaaaaA
        timer.reset();
//AaaaAAAAA
        timer.start();
//AaAaAAAAaAAaAaaaaaa

//aaAaaAaaA
        if (shouldReset)
//aaAaAAAAaaAaaaaaAaa
            robotState.forceRobotPose(trajectory.getInitialState().poseMeters);
//AAaaAAaaaaaAAAaAaaa
        //drive.resetOdometry(new Pose2d(pathState.poseMeters.getTranslation(), pathState.holonomicRotation));
//AaaAaaaAAA

//aaAaAaAaa
    }
//aaaAAAAaaaaAaaa

//AaAAaaAAA
    @Override
//aAAAAaaAAa
    public void execute() {
//aaAaaaaaaaaAaaAAa
        
//aaAaaaAaaaa
        if (DriveConstants.followTrajectoryXControllerKp.hasChanged())
//AAAAAaAaaaaaaAaAaAa
            xController.setP(DriveConstants.followTrajectoryXControllerKp.get());
//AaaAAA
        if (DriveConstants.followTrajectoryXControllerKd.hasChanged())
//AAaAaAa
            xController.setD(DriveConstants.followTrajectoryXControllerKd.get());
//AaaaAaAa

//AAAaAAaaAAAa
        if (DriveConstants.followTrajectoryYControllerKp.hasChanged())
//AaAAAaaAaaaaaAAA
            yController.setP(DriveConstants.followTrajectoryYControllerKp.get());
//AaAAAAaAA
        if (DriveConstants.followTrajectoryYControllerKd.hasChanged())
//AaAAAAa
            yController.setD(DriveConstants.followTrajectoryYControllerKd.get());
//AaAaAAaAAa
        
//aaAaAaAAAAAaaA
        if (DriveConstants.followTrajectoryThetaControllerKp.hasChanged())
//aAAAAAaAAa
            thetaController.setP(DriveConstants.followTrajectoryThetaControllerKp.get());
//aaaAAAAAAAaAaAaA
        if (DriveConstants.followTrajectoryThetaControllerKd.hasChanged())
//aAAaAaaa
            thetaController.setD(DriveConstants.followTrajectoryThetaControllerKd.get());
//aaaaaA

//aAaAaaaAaAa
        latestFieldToVehicle = robotState.getLatestFieldToVehicle();
//aaaAaAAAaAaAaAAAAAa
        
//AaaaAAaAAA
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());
//AaaAAaaAaaa

//AaaAAAAAaAA
        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
//AAaaAAaAaaaaAaaAaa
        /*if (timer.get() / trajectory.getTotalTimeSeconds() >= 0.75 && vision.hasTarget()) {
//aAAaaAaaaaaA
            double omegaOut = xController.calculate(vision.getTX(), 0);
//AaaAaAaaAAaaa
            desiredState.poseMeters = new Pose2d(0, desiredState.poseMeters.getY(), new Rotation2d(omegaOut));
//AAAAAaAaaaa
            adjustedSpeeds = controller.calculate(
//AAAAaAaaaAaaaAa
                latestFieldToVehicle, desiredState, desiredState.holonomicRotation);
//aaAAAAa
        }
//aAaaaaaaAaA
        else {*/
//aaaAaAaAaaAAAAaa
            adjustedSpeeds = controller.calculate(
//AAAAAAAaAAAAA
                latestFieldToVehicle, desiredState, desiredState.holonomicRotation);
//aaaaaaAAAAAAaaAAaaA
        //}
//aaaAAaaAaaaAAaa

//AAAaa
        drive.setGoalChassisSpeeds(adjustedSpeeds);
//AAAAaAaa
    }
//AaAAa

//aaAaaAaAA
    @Override
//AAAaaAaAaAaaaA
    public boolean isFinished() {
//AaAaAAaAAA
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
//AaaAAaAAaaaaa
    }
//AaAaAaAa

//AaAAA
    @Override
//aAaAaaAAa
    public void end(boolean interrupted) {
//AAaaAAAAAAa
        timer.stop();
//aaaAAAAaAAaA
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
//AAAaaAAaAAaaAaAA
    }
//AAAAAa

//aaaaAaaaAAaAAaa
}