package frc.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.BaseFalconSwerveConstants.CANCODER_INVERT;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.BaseFalconSwerveConstants.ANGLE_MOTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.BaseFalconSwerveConstants.ANGLE_NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.BaseFalconSwerveConstants.ANGLE_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.BaseFalconSwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.BaseFalconSwerveConstants.ANGLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.BaseFalconSwerveConstants.ANGLE_CURRENT_THRESHOLD;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.BaseFalconSwerveConstants.ANGLE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.BaseFalconSwerveConstants.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.BaseFalconSwerveConstants.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.BaseFalconSwerveConstants.ANGLE_KD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.BaseFalconSwerveConstants.DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.BaseFalconSwerveConstants.DRIVE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.BaseFalconSwerveConstants.DRIVE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.BaseFalconSwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.BaseFalconSwerveConstants.DRIVE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.BaseFalconSwerveConstants.DRIVE_CURRENT_THRESHOLD;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.BaseFalconSwerveConstants.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.BaseFalconSwerveConstants.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.BaseFalconSwerveConstants.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.BaseFalconSwerveConstants.DRIVE_KD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.BaseFalconSwerveConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.BaseFalconSwerveConstants.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.BaseFalconSwerveConstants.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.BaseFalconSwerveConstants.CLOSED_LOOP_RAMP;
    }
}