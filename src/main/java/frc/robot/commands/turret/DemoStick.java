package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.Turret;

public class DemoStick extends CommandBase {
    
    private final Vision vision;
    private final Turret turret;
    private final DoubleSupplier xS;
    private final DoubleSupplier yS;

    private final LinearFilter filter = LinearFilter.singlePoleIIR(0.05, 0.02);

    private double goal = 0;

    public DemoStick(Vision vision, Turret turret, DoubleSupplier x, DoubleSupplier y) {
        this.vision = vision;
        this.turret = turret;
        this.xS = x;
        this.yS = y;

        addRequirements(vision, turret);
    }

    @Override
    public void initialize() {
        filter.reset();
        vision.turnOffLeds();
    }

    @Override
    public void execute() {

        double x = xS.getAsDouble();
        double y = -yS.getAsDouble();

        if ((Math.abs(x) > 0.5 || Math.abs(y) > 0.5)) {
            if (y == 1)
                y -= 0.05;
            else if (y == -1)
                y += 0.05;
            double rot = Math.atan(y / x);
            if (x < 0)
                rot -= Math.PI;
            rot += Math.PI / 2;
            rot = filter.calculate(rot);
            goal = rot;
            //SmartDashboard.putNumber("DemoRot", Units.radiansToDegrees(rot));
        }
        turret.setPositionGoal(new Rotation2d(goal - RobotState.getInstance().getLatestFieldToVehicle().getRotation().getRadians()), 0);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        vision.turnOffLeds();
    }
}
