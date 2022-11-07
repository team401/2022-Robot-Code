package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretClimbPosition extends CommandBase {

    private final Turret turret;
    private final Vision vision;
    private final Rotation2d desiredRotation;

    public TurretClimbPosition(Turret turret, Vision vision) {

        this.turret = turret;
        this.vision = vision;
        this.desiredRotation = new Rotation2d(Math.PI / 2);

        addRequirements(turret, vision);

    }

    @Override
    public void initialize() {

        vision.turnOffLeds();
        turret.setPositionGoal(desiredRotation);

    }

    @Override
    public void end(boolean isInterrupted) {

        turret.setVoltage(0);
        vision.turnOnLeds();

    }
    
}
