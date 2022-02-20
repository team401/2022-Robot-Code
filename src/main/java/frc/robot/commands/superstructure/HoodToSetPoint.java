package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodToSetPoint extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private double setPoint;

    public HoodToSetPoint(ShooterSubsystem shooter) {

        shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void execute() {

        setPoint = SmartDashboard.getNumber("Hood SetPoint", 5);

        shooterSubsystem.hoodSetDesiredClosedStateRevolutions(setPoint);

    }


    
}
