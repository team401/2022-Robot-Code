package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends SwerveControllerCommand {
    
    /**
     * Command that is used to follow a given trajectory (which we almost will exclusively use for 
     * autonomous pathing)
     * Uses SwerveControllerCommand class, which uses HolonomicDriveController
     */

    private final Trajectory trajectory;
    private Timer timer = new Timer();
    private DriveSubsystem drive;
    
    /**
     * PID Controller that also makes a trapezoid shape since we put constraints on how fast
     * we can go and how fastly we can ramp up and down to speeds (makes a trapezoid when you
     * look at it)
     */
    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(
            3.0, 
            0.0, 
            0.0, 
            new TrapezoidProfile.Constraints(
                AutoConstants.maxVelocityMetersPerSec, 
                AutoConstants.maxAccelerationMetersPerSecondPerSecond
            )
        );

    public FollowTrajectory(DriveSubsystem subsystem, Trajectory toFollow) {

        /**
         * We use the Super Constructor to create the object (parameters are listed in order):
         * Trajectory we want to follow
         * Method reference to the pose supplier (found in the Drive Subsystem)
         * Drive kinematics (found in constants)
         * X PID Controller
         * Y PID Controller
         * Rotation Controller (initialized earlier)
         * Method reference to how we control our modules (found in the Drive Subsystem)
         * our requirements (basically which subsystem we are using)
         */

        super (
            toFollow,
            subsystem::getPose,
            DriveConstants.kinematics,
            new PIDController(2.45, 0, 1.0), //check these values!
            new PIDController(2.45, 0, 1.0), //check these values!
            rotationController,
            subsystem::setModuleStates,
            subsystem
        );
        //initialize our variables
        drive = subsystem;
        trajectory = toFollow;

        //set our rotation controller to wrap from [-pi, pi]
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }

    //we initialize by reseting and starting our timer, and then going to the super method
    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        super.initialize();

    }

    //our execute method just calls the super method
    //technically might not need it?
    @Override
    public void execute() {

        super.execute();

    }

    //gets the initial pose, useful for setting our starting position when following a trajectory
    public Pose2d getInitialPose() {

        return trajectory.getInitialPose();

    }

}
