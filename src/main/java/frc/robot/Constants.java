package frc.robot;

//import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.NEOSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final boolean collectorTuningMode = false;
    public static final boolean tunePivot = true;
    public static final boolean tuneSwerve = true;
    public static final boolean tuneLauncher = false;
    public static final double stickDeadband = 0.3;


    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final NEOSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            NEOSwerveConstants.SDSMK4i(NEOSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelDiameter = chosenModule.wheelDiameter;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final double voltageComp = 12.0;


        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean absoluteEncoderPortsInvert = chosenModule.absoluteEncoderPortsInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 20;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 45;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        // public static final double angleKP = chosenModule.angleKP;
        // public static final double angleKI = chosenModule.angleKI;
        // public static final double angleKD = chosenModule.angleKD;
        // public static final double angleKFF = chosenModule.angleKFF;

        public static final double angleKP = 0.05;
        public static final double angleKI = 0.00;
        public static final double angleKD = 0.02;
        public static final double angleKFF = 0.00;


        /* Drive Motor PID Values */
        public static final double driveKP = 0.25; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.3;
        public static final double driveKFF = 0.0;


        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.2670125); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.456125);
        public static final double driveKA = (0.387935);

        public static final double driveConversionPositionFactor = 
            (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        /** Meters per Second */
        
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 11.5; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast; //set to brake
        public static final IdleMode driveNeutralMode = IdleMode.kBrake; //set to brake
       // public static final double angleConversionFactor = 0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int absoluteEncoderPorts = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(((0.009)*360));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int absoluteEncoderPorts = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(((0.96)*360) + 182.0); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int absoluteEncoderPorts = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int absoluteEncoderPorts = 11; //MIGHT SWAP WITH 1
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(47); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
