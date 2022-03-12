package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    private final TurretIOInputs inputs = new TurretIOInputs();
    private PIDController positionController = new PIDController(TurretConstants.positionKp.get(), 0, TurretConstants.positionKd.get());
    private final TurretIO io;
    private Rotation2d goalPosition = new Rotation2d();
    private double velocityGoal = 0;

    public Turret(TurretIO io) {
        this.io = io;
        

        io.resetEncoderAbsolute();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Turret", inputs);

        // Update gains if they have changed
        if (TurretConstants.positionKp.hasChanged() || TurretConstants.positionKd.hasChanged()) {
            positionController.setP(TurretConstants.positionKp.get());
            positionController.setD(TurretConstants.positionKd.get());
        }

        if (TurretConstants.velocityKp.hasChanged() || TurretConstants.velocityKd.hasChanged()) {
            io.setVelocityPD(TurretConstants.velocityKp.get(), TurretConstants.velocityKd.get());
        }

        Rotation2d turretRotation = new Rotation2d(MathUtil.angleModulus(inputs.positionRad));
        Logger.getInstance().recordOutput("Turret/RotationDeg", turretRotation.getDegrees());
        Logger.getInstance().recordOutput("Turret/SetpointDeg", goalPosition.getDegrees());
        Logger.getInstance().recordOutput("Turret/VelocityFFDegPerSec", Units.radiansToDegrees(velocityGoal));

        //PID control - equivalent of our old setdesiredpositionclosedloop methods continuously
        double output = positionController.calculate(turretRotation.getRadians(), goalPosition.getRadians());
        output += TurretConstants.turretModel.calculate(velocityGoal);
        io.setVoltage(output);

        RobotState.getInstance().recordTurretObservations(turretRotation, inputs.velocityRadPerS);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPositionGoal(Rotation2d goal, double velocity) {

        velocityGoal = velocity;
        double goalWrapped = MathUtil.angleModulus(goal.getRadians());
        
        //clamps max values to be within -90 and 90 deg
        goalWrapped = MathUtil.clamp(goalWrapped, -Math.PI / 2, Math.PI / 2);
        this.goalPosition = new Rotation2d(goalWrapped);
    }

    public void setPositionGoal(Rotation2d goal) {

        setPositionGoal(goal, 0);

    }

    public double getVelocityRadPerS() {
        return inputs.velocityRadPerS;
    }


}
