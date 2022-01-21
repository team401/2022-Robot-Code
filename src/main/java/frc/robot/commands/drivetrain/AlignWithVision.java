package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithVision extends CommandBase {

    /**
     * Uses the limelight to align ourselves horizontally with the target by only changing the rotation
     * commanded values
     */

    //the subsystems we need to initialize
    private final DriveSubsystem drive;
    private final VisionSubsystem limelight;

    //our controller to get us to our desired orientation
    private final PIDController rotVisionPIDController = new PIDController(
        0.0, 
        0.0, 
        0.0
    );

    //timer to keep track of progression (to make sure we do base our finished state on a set of data)
    private Timer timer = new Timer();

    //what we will command to the drive
    private double rotationOut = 0.0;

    //tolerance of how many degrees we will allow ourselves to be off by
    private double tolerance = Units.degreesToRadians(1.5);

    //constructor
    public AlignWithVision(DriveSubsystem subsystem, VisionSubsystem vision){

        //initialize the subsystems
        drive = subsystem;
        limelight = vision;

        //add requirements to list the subsystems we are currently using
        
        addRequirements(drive, limelight);

    }

    @Override
    public void initialize() {

        //turn on the limelight
        limelight.setLedMode(0);

        //reset + start timer
        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        //do we see something?
        if (limelight.hasValidTarget()){

            //is there a noticeable difference greater than our tolerance?
            if (Math.abs(limelight.gettX()) > tolerance) {

                //calculate what to send based on our current reading
                rotationOut = rotVisionPIDController.calculate(limelight.gettX(), 0.0);

            } else {

                //we don't need to change anything with our current rotation
                rotationOut = 0.0;

            }

            /**
             * Command the drivetrain to drive using the commanded forward and strafe, and the newly
             * calculated rotation values based on the PID
             */
            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative()
            );

        } else {

            //reset timer to say that we have lost target
            timer.reset();

            //use the given drive values to send to the drivetrain
            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                drive.getCommandedDriveValues()[2], 
                drive.getIsFieldRelative()
            );

        }

    }

    @Override
    public boolean isFinished() {

        /**
         * we are only finished if we are currently seeing a valid target, are within tolerance, and 
         * have been seeing a target for the past 500 ms
         */
        return limelight.hasValidTarget() && (Math.abs(limelight.gettX()) < tolerance && timer.get() >= 0.5);

    }

    @Override
    public void end(boolean interrupted){

        //turns off the Limelight LEDs once we are done
        limelight.setLedMode(1);

    }
    
}
