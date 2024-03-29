package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.RotationArms;
import frc.robot.subsystems.Tower;

public class Intake extends CommandBase {
    
    private final IntakeWheels intake;
    private final Tower tower;
    private final RotationArms rotationArms;
    
    private final Timer lastSensorUpdateTimer = new Timer();
    private int lastSensorRed = 0;

    private final Timer reverseTimer = new Timer();

    public Intake(Tower tower, IntakeWheels intake, RotationArms rotationArms) {

        this.tower = tower;
        this.intake = intake;
        this.rotationArms = rotationArms;

        addRequirements(tower, intake);
    }

    @Override
    public void initialize() {
        lastSensorUpdateTimer.reset();
        lastSensorUpdateTimer.start();

        reverseTimer.reset();
        reverseTimer.stop();
    }

    @Override
    public void execute() {

        if (!tower.getTopSensor()) tower.setConveyorPercent(BallConstants.towerPower);
        else tower.setConveyorPercent(0.0);

        tower.setIndexWheelsPercent(BallConstants.towerPower);
        intake.setPercent(BallConstants.intakePower.get());
        
    }

    @Override
    public void end(boolean isInterrupted) {

        tower.setConveyorPercent(0.0);
        tower.setIndexWheelsPercent(0.0);
        intake.setPercent(0.0);

    }

}