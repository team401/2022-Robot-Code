package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.util.Interpolation.InterpolatingDouble;

public class DemoShoot extends CommandBase {

    private final Shooter shooter;
    private final Tower tower;
    private final DoubleSupplier trigger;

    public DemoShoot(Shooter shooter, Tower tower, DoubleSupplier trigger) {

        this.shooter = shooter;
        this.tower = tower;
        this.trigger = trigger;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        double rpm = SmartDashboard.getNumber("MaxShooterRPM", 2000) * trigger.getAsDouble();
        SmartDashboard.putNumber("RPM", rpm);
        double hood = SmartDashboard.getNumber("DesiredHoodPosition(0.27-0.63)", 0.63);
        SmartDashboard.putNumber("HOOD", hood);
        shooter.setSetpoint(hood, Units.rotationsPerMinuteToRadiansPerSecond(rpm));
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}