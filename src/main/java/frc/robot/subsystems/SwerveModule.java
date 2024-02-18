package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.math.OnBoardModuleState;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.lib.math.Conversions;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private AnalogInput angleEncoder;
    private RelativeEncoder driveEncoder;


    private RelativeEncoder integratedAngleEncoder;
    private final SparkPIDController angleController;
    private final SparkPIDController driveController;

    private final MutableMeasure<Voltage> mutableAppliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> mutableDistance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> mutableVelocity = mutable(MetersPerSecond.of(0));


    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new AnalogInput(moduleConstants.absoluteEncoderPorts);

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();

        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = OnBoardModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    
    public void resetToAbsolute()  {
        double absolutePosition = getAbsoluteEncoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition); 
        
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
       return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getAbsoluteEncoder(){
        double absolutePositionDegrees = angleEncoder.getVoltage() / RobotController.getVoltage5V() * 360;
        return Rotation2d.fromDegrees(absolutePositionDegrees);
    }

    private void configAngleMotor(){
        mAngleMotor.clearFaults();
        mAngleMotor.restoreFactoryDefaults(); 
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        //the above is fixed in the constants file
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF); //fixed in constannts file
        // angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp); refer to the note below
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){ 
        mDriveMotor.clearFaults();       
        mDriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor); 
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor); 
        driveController.setP(Constants.Swerve.driveKP);
        driveController.setI(Constants.Swerve.driveKI);
        driveController.setD(Constants.Swerve.driveKD);
        driveController.setFF(Constants.Swerve.driveKFF); 
        mDriveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);        
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }



    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getAngle()
        );
    }

    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getDriveAppliedOutput() {
        return mDriveMotor.getAppliedOutput();
    }

    public void setDriveVoltage(double volts) {
        mDriveMotor.setVoltage(volts);
    }

    public void openLoopDiffDrive(double voltage) {
        angleController.setReference(0, ControlType.kPosition);
        mDriveMotor.setVoltage(voltage);
    }

    public void logMotor(SysIdRoutineLog log) {
        log.motor("module# " + moduleNumber)
            .voltage(mutableAppliedVoltage.mut_replace(
                mDriveMotor.getAppliedOutput() * mDriveMotor.getBusVoltage(), Volts))

            .linearVelocity(mutableVelocity.mut_replace(driveEncoder.getVelocity(), MetersPerSecond))
            .linearPosition(mutableDistance.mut_replace(driveEncoder.getPosition(), Meters));
    }

}