package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class TeleopPivot extends Command {
    
    private Pivot pivot;
    private DoubleSupplier translationSupplier;

    public TeleopPivot(Pivot pivot, DoubleSupplier translationSupplier) {
        this.pivot = pivot;
        addRequirements(pivot);  

        this.translationSupplier = translationSupplier;
    }

     @Override
     public void execute() {
         pivot.movePivot(
            Math.pow(
                MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.stickDeadband),
                3));
     }
}
