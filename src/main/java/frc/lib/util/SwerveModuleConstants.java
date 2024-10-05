package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final int ANGLE_MOTOR_ID;
    public final int CANCODER_ID;
    public final Rotation2d ANGLE_OFFSET;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param DRIVE_MOTOR_ID
     * @param ANGLE_MOTOR_ID
     * @param CANCODER_ID
     * @param ANGLE_OFFSET
     */
    public SwerveModuleConstants(int DRIVE_MOTOR_ID, int ANGLE_MOTOR_ID, int CANCODER_ID, Rotation2d ANGLE_OFFSET) {
        this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
        this.ANGLE_MOTOR_ID = ANGLE_MOTOR_ID;
        this.CANCODER_ID = CANCODER_ID;
        this.ANGLE_OFFSET = ANGLE_OFFSET;
    }
}
