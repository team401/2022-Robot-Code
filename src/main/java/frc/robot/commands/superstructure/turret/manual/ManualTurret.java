package frc.robot.commands.superstructure.turret.manual;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 *TO DO:
 * It will stay fixed on one position in cases of emergency, which will default to 0 degrees (middle 
 * of the turret) 
 * Does not use the limelight
 */

public class ManualTurret extends CommandBase {

        //Subsystems
        private final LimelightSubsystem limelightSubystem;
        private final TurretSubsystem turretSubsystem;
    
        //Button Values
        private BooleanSupplier moveLeft;
        private BooleanSupplier moveRight;
    
        public ManualTurret(LimelightSubsystem limelight, TurretSubsystem turret, 
            BooleanSupplier left, BooleanSupplier right) {
    
            limelightSubystem = limelight;
            turretSubsystem = turret;
    
            moveLeft = left;
            moveRight = right;
    
            addRequirements(limelightSubystem, turretSubsystem);
    
        }
    
        @Override
        public void initialize() {
    
            SmartDashboard.putBoolean("ManualTurret Running", true);
    
        }
    
        @Override
        public void execute() {
    
              if (moveLeft.getAsBoolean()) {
    
                turretSubsystem.runTurretPercent(-0.2);
    
              }
              else if (moveRight.getAsBoolean()) {
    
                turretSubsystem.runTurretPercent(0.2);
                
              }
              else {
    
                turretSubsystem.runTurretPercent(0);
    
              }
        }
    
        @Override
        public void end(boolean isInterrupted) {
            SmartDashboard.putBoolean("ManualTurret Running", false);
        }
    
    
        
    }
    
    

