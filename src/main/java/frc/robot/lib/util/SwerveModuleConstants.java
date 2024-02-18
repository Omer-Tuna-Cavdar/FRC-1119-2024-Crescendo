package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int absoluteEncoderPorts;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param absoluteEncoderPorts
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int absoluteEncoderPorts, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.absoluteEncoderPorts = absoluteEncoderPorts;
        this.angleOffset = angleOffset;

    }
}