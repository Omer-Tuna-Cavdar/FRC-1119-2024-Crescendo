package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnAndX extends Command {
    
    private Swerve swerve;
    private double rotationVal;

private PIDController thetaController;
    public TurnAndX(Swerve swerve){
        this.swerve = swerve;
               

        addRequirements(swerve);
            }

    @Override
    public void initialize() {
     thetaController = new PIDController(.05, 0, 0.2);
    thetaController.enableContinuousInput(-180, 180);
    thetaController.setTolerance(10);
    }

    @Override
    public void execute() {
        thetaController.setSetpoint(45);
        rotationVal =  thetaController.calculate(MathUtil.inputModulus(swerve.getPose().getRotation().getDegrees(), -180, 180), thetaController.getSetpoint()); 
        rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxAngularVelocity * 0.075, Constants.Swerve.maxAngularVelocity * 0.075);
        
         swerve.drive(new Translation2d(0,0), rotationVal, true, true);
        // if (thetaController.atSetpoint()) {
        // swerve.drive(new Translation2d(0,0), 0, true, true);
        // swerve.xPosition(true);
        // }

    }

    @Override
    public boolean isFinished() {     
           if (thetaController.atSetpoint()) {
            return true;
    } else {
        return false;
    }
    
    }

    @Override
    public void end(boolean interrupted) {
        swerve.xPosition(true);
    }

}
