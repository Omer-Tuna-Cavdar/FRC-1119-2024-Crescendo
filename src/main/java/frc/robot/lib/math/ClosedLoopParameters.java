package frc.robot.lib.math;

public class ClosedLoopParameters {

    public double kP, kI, kD, kF, kS, kV, kA;

    public ClosedLoopParameters(double kP, double kI, double kD, double kF, double kS, double kV, double kA){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
    
}
