package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSupplier;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldCentricSupplier) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldCentricSupplier = fieldCentricSupplier;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
        double strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 3);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !fieldCentricSupplier.getAsBoolean(), 
            true
        );
    }
}