package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class TuningConstants { // Basically stuff you have to tune

        /* For these, align all the wheels so their gears are facing toward RSL Side */

        public static final double CANCoder0_zero = -47.81, // Front Left
                                   CANCoder1_zero = -47.15, // Front Right
                                   CANCoder2_zero = -128.5, // Back Left
                                   CANCoder3_zero = -88; // Back Right
        
        public static final boolean useCANCoders = true;
        
        /* Swerve Drive Constants */

        public static final double drive_motor_p = 0.05,
                                   drive_static_voltage = 0.32, 
                                   drive_equilibrium_voltage = 1.51, 
                                   drive_acceleration_voltage = 0.27, // SYSID values: KS, KV, KA; they are automatically divided by 12 later
                                   max_linear_speed = 10, // feet per second; theoretical max is 13.5
                                   max_angular_speed = 360; // degrees per second; theoretical max is theoretical maximum is max_linear_speed * 46.6436741705 which is roughly 629.6896013018

        /* PathPlanner PID Constants */

        public static final double autonomous_max_linear_speed = 10,
                                   autonomous_ramp_up_time_linear = 0.75, // in seconds to reach max 
                                   autonomous_max_angular_speed = 400, 
                                   autonomous_ramp_up_time_angular = 0.66,
                                   autonomous_translation_p_controller = 2, 
                                   autonomous_angle_p_controller = 4;
        
        /* Driving PID Constants */

        public static final double teleop_angle_p = 0.165,
                                   teleop_angle_i = 0,
                                   teleop_angle_e = 1.35,
                                   teleop_translation_p = 0.4, 
                                   teleop_translation_i = 0,
                                   teleop_translation_tolerance = 1;
        
        public static final double turn_slow_ratio = 1; // because slowing down rotation and translation by the same factor is insane
                                    // ex. it its 4, then 0.6 translation ratio goes to a 0.9 turning ratio
        
        /* Pivot Constants */

        public static final double pivot_p = .015, 
                                   pivot_i = 0, 
                                   pivot_g = 0.018, 
                                   pivot_e = 1.05, 
                                   pivot_zero = 115;
        
        public static final double starting_angle = -9.255644, // thanks to certain people, it's not -9.255644, // all in degrees
                                   intake_angle = 59.006106, 
                                   amp_angle = 125.388397, 

                                   min_outtake_angle = 25, 
                                   pivot_guard_angle = 0; // so we dont smack against the walls. goal is to get this to 0 soon
        
        /* Hang Constants */
        
        public static final double hang_p = 0.0, // temporarily disabled
                                   hang_i = 0.0, 
                                   hang_e = 1.0;
        
        /* Intaking Constants */

        public static final double acceleration_time = 1, 
                                   min_intake_time = 8, 
                                   add_intake_time = 0.8, 
                                   add_conveyor_time = 0.2, 
                                   retract_conveyor_time = 0.15; // how much we intake after the note has passed through
        
        /* Outtaking Constants */

        public static final double max_flywheel_acceleration_time = 4, 
                                   min_outtake_time = 1, 
                                   min_amp_time = 2;

    }


    public static final class Buttons {

        /* D-pad */

        public static final int dpad_up = 0, 
                                dpad_right = 90, 
                                dpad_down = 180, 
                                dpad_left = 270;

        /* Which USB port is used for the different controllers */

        public static final int operator_usb_port = 1,
                                driver_usb_port = 2;

        /* Driver Buttons */

        public static final int xBoxTranslationAxis = XboxController.Axis.kLeftY.value, 
                                xBoxStrafeAxis = XboxController.Axis.kLeftX.value,

                                xBoxRotationAxis = XboxController.Axis.kRightX.value,

                                xBoxDirectionXAxis = XboxController.Axis.kRightX.value, 
                                xBoxDirectionYAxis = XboxController.Axis.kRightY.value, 
                                xBoxTurnRightAxis = XboxController.Axis.kRightTrigger.value, 
                                xBoxTurnLeftAxis = XboxController.Axis.kLeftTrigger.value, 
                                
                                xBoxSlowAxis = XboxController.Axis.kRightTrigger.value, 
                                xBoxSlowButtonOne = XboxController.Button.kRightBumper.value, 
                                xBoxSlowButtonTwo = XboxController.Button.kLeftBumper.value, 

                                xBoxRobotCentricButton = XboxController.Button.kLeftBumper.value,

                                xBoxZeroGyroButton = XboxController.Button.kY.value, 
                                xBoxMakeXButton = XboxController.Button.kX.value, 
                                xBoxTerminateCommandsDriverButton = XboxController.Button.kBack.value,

                                joystickTranslationAxis = 1, 
                                joystickStrafeAxis = 0, 

                                joystickRotationAxis = 5, 

                                joystickSlowButton = 1,

                                joystickDirectAngleButton = 2, 

                                joystickZeroGyroButton = 4, 
                                joystickMakeXButton = 2, 
                                joystickDriveToAmpButton = 4; // if we push the slider forward it cancels commands

        /* Operator Buttons */

        public static final int terminateCommandsOperatorButton = XboxController.Button.kBack.value;
    }


    public static final class GeneralConstants {

        public static final boolean relative_drive = true;

        public static final double joystick_deadzone = 0.2; // The deadband value to use on any joystick

        public static final double axis_exponent = 1.3;
        
        public static final double starting_yaw = 0; // shoot this might depend based on autonomous... we can make a new command xoxo

        // TODO: ONLY constant values should be in this file, NOT functions!  These
        // should get put into a separate place or inlined where they are used!

        /**
         * Calculates the signed power of the given axis value.
         *
         * This function applies a deadband to the input value to reduce noise and jitter.
         * It then calculates the power of the absolute value of the input value using the
         * specified exponent. The sign of the output is determined by the sign of the input value.
         *
         * @param axis_value The input axis value.
         *
         * @return The signed power of the input value.
         */
        public static final double signedPower(double axis_value) {
            // Apply a deadband to the value so that any values in the deadband become 0
            axis_value = MathUtil.applyDeadband(axis_value, joystick_deadzone);

            // If the value is 0 we are done
            if (axis_value == 0)
                return 0;

            // Calculate the signed power.  If the value is negative, then we want
            // the returned value to also be negative.
            if (axis_value < 0)
                return 0 - Math.pow(0 - axis_value, axis_exponent);

            // Otherwise we return the positive signed power
            return Math.pow(axis_value, axis_exponent);
        }


        /**
         * Normalizes an angle to the range of -180 to 180 degrees (in theory) but it may not quite
         * do this for negative angles (see detailed comments inside).
         *
         * This function ensures that the input angle is always represented in the range of -180 to 180 degrees (UNTRUE).
         * It is useful for applications that deal with angles or rotations.
         *
         * @param degrees The input angle in degrees.
         *
         * @return The normalized angle in the range of -180 to 180 degrees.
         */
        public static double normalizeAngle(double degrees) {
            // If the input angle degrees is negative, subtract 180 degrees then modulo it for 360 degrees and
            // then add 180 degrees. This effectively maps a negative angle to a positive angle in the range
            // of 180 to 360 degrees; NOT -180 to 180 or -180 to 0.
            if (degrees < 0)
                return ((degrees - 180) % 360 + 180);

            // If the input angle is positive, add 180 degrees then modulo it for 360 degrees and then subtract
            // 180 degrees.  This effectively maps a non-negative angle to a negative angle in the range
            // of 0 to -180 degrees.
            return ((degrees + 180) % 360 - 180);
        }


        /**
         * Calculates a Proportional-Exponent (PE) correction value based on the given error.
         *
         * This function is commonly used in control systems to determine a corrective output based on
         * the error between the desired and actual values. It uses a proportional and exponential
         * relationship to calculate the correction.
         *
         * @param error The error between the desired and actual values.
         * @param proportional The proportional constant.
         * @param exponent The exponent for the error calculation.
         * @param min_power The minimum power value.
         * @param max_power The maximum power value.
         *
         * @return The calculated PE correction value.
         */
        public static final double getPECorrection(double error, double proportional, double exponent, double min_power, double max_power) {
            // If the error is zero no correction is needed and we are done here.
            if (Math.abs(error) == 0)
                return 0;

            // Set the multiplier to the max_power if the error is positive and to the negative of max_power if
            // the error is negative. This ensures the correction is applied in the correct direction.
            double multiplier = max_power;
            if (error < 0) {
                error = 0 - error;
                multiplier = -max_power;
            }

            // Calculate the proportional error
            error *= proportional / max_power; // as a proportion of maximum power

            // Clamp the error at 1 to prevent over correction
            error = Math.min(error, 1);

            // Calculate the power with the proper magnitude and direction of the correction.
            double calculated_power = Math.pow(error, exponent) * multiplier;

            // Return 0 if the absolute value of the calculated power is less than the min_power to avoid small,
            // insignificant corrections. Otherwise, return the calculated power.
            return (Math.abs(calculated_power) < min_power) ? 0 : calculated_power;
        }

        // TODO: Change to return nothing since no callers in this code check the return value.
        /** Returns false if the key already exists with a different type */
        public static final boolean log(String key, String value) {
            return SmartDashboard.putString(key, value);
        }

        // TODO: Change to return nothing since no callers in this code check the return value.
        /** Returns false if the key already exists with a different type */
        public static final boolean log(String key, String[] value) {
            return SmartDashboard.putStringArray(key, value);
        }
        
        // TODO: Change to return nothing since no callers in this code check the return value.
        /** Returns false if the key already exists with a different type */
        public static final boolean log(String key, double value) {
            return SmartDashboard.putNumber(key, value);
        }
        
        // TODO: Change to return nothing since no callers in this code check the return value.
        /** Returns false if the key already exists with a different type */
        public static final boolean log(String key, double[] value) {
            return SmartDashboard.putNumberArray(key, value);
        }
        
        // TODO: Change to return nothing since no callers in this code check the return value.
        /** Returns false if the key already exists with a different type */
        public static final boolean log(String key, boolean value) {
            return SmartDashboard.putBoolean(key, value);
        }
        
        // TODO: Change to return nothing since no callers in this code check the return value.
        /** Returns false if the key already exists with a different type */
        public static final boolean log(String key, boolean[] value) {
            return SmartDashboard.putBooleanArray(key, value);
        }
    }

    public static final class BaseFalconSwerveConstants {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.75); // 28" width -> 22.75" track width
        public static final double wheelBase = Units.inchesToMeters(18.75); // 24" drivetrain length -> 18.75" wheel base
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 35;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 35;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = TuningConstants.drive_motor_p; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (TuningConstants.drive_static_voltage / 12.0);
        public static final double driveKV = (TuningConstants.drive_equilibrium_voltage / 12.0);
        public static final double driveKA = (TuningConstants.drive_acceleration_voltage / 12.0);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = TuningConstants.max_linear_speed * 0.3048;
        /** Radians per Second */
        public static final double maxAngularVelocity = TuningConstants.max_angular_speed * Math.PI / 180.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder0_zero + 90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder1_zero + 90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder2_zero + 90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 11; // bc they were messed up from the can id thing
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder3_zero + 90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = TuningConstants.autonomous_max_linear_speed * 0.3048;
        public static final double kMaxAccelerationMetersPerSecondSquared = TuningConstants.autonomous_max_linear_speed * 0.3048 / TuningConstants.autonomous_ramp_up_time_linear;
        public static final double kMaxAngularSpeedRadiansPerSecond = TuningConstants.autonomous_max_angular_speed * Math.PI / 180.0;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = TuningConstants.autonomous_max_angular_speed * Math.PI / 180.0 / TuningConstants.autonomous_ramp_up_time_angular;
    
        public static final double kPXController = TuningConstants.autonomous_translation_p_controller;
        public static final double kPYController = TuningConstants.autonomous_translation_p_controller;
        public static final double kPThetaController = TuningConstants.autonomous_angle_p_controller;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class TeleopSwerveConstants {
        public static final double swerve_min_pid_rotation = 0.05,
                                   swerve_max_pid_rotation = 0.8, 
                                   swerve_calibration_time = 0.5, 
                                   swerve_min_manual_translation = 0.05, 
                                   swerve_min_manual_rotation = 0.02,

                                   teleop_swerve_slow_factor = 0.2, 
                                   
                                   vision_tolerance = 5, 
                                   
                                   teleop_rotation_percent = 0.75, 
                                   swerve_bumper_turn_sensitivity = 0.35; // ratio of teleop swerve rotation speed vs maximum swerve rotation speed
    }
    
    public static final class IntakeConstants {

        /* CAN IDs */

        public static final int intake_motor_id = 20; // Neo
        
        /* Motor Parameters */

        public static final MotorType intake_motor_type = MotorType.kBrushless;
        
        public static final boolean intake_motor_clockwise_positive = true, 
                                    intake_motor_brake = true;
        
        public static final int intake_motor_max_continuous_current = 15;

        public static final double intake_motor_max_current = 25, 

                                   intake_motor_max_percent_output_per_second = 3, 

                                   intake_motor_free_current = 11;

        /* Subsystem Variables */

        public static final double intake_motor_speed = 1.0;
        
    }

    public static final class ConveyorConstants {

        /* CAN IDs */

        public static final int conveyor_motor_id = 19; // Neo
        
        /* Motor Parameters */
        
        public static final MotorType conveyor_motor_type = MotorType.kBrushless;

        public static final boolean conveyor_motor_clockwise_positive = true, 
                                    conveyor_motor_brake = true;
        
        public static final int conveyor_motor_max_continuous_current = 10;

        public static final double conveyor_motor_max_current = 15, 
                                           
                                   conveyor_motor_max_percent_output_per_second = 3,

                                   conveyor_motor_free_current = 5;

        /* Subsystem Variables */

        public static final double conveyor_intake_speed = 0.7, 
                                   conveyor_retract_speed = -0.5, 
                                   conveyor_outtake_speed = 0.8, 
                                   conveyor_amp_speed = 0.8;

    }

    public static final class PivotConstants {

        /* CAN IDs */

        public static final int pivot_motor_ID = 13; // Kraken

        /* Motor Parameters */

        public static final boolean pivot_motor_clockwise_positive = false, 
                                    pivot_motor_brake = false, 
                                    pivot_motor_invert_sensor = false;
                
        public static final double pivot_motor_max_continuous_current = 15, 
                                   pivot_motor_max_current = 25, 

                                   pivot_motor_min_percent_output = 0, 
                                   pivot_motor_max_percent_output = 0.95, 
                                   
                                   pivot_motor_max_percent_output_per_second = 10, 
                                   
                                   pivot_motor_gear_ratio = (5.0 / 1.0) * (9.0 / 1.0) * (34.0 / 10.0), 
                                   
                                   pivot_motor_calibration_time = 0.2, 

                                   pivot_motor_free_current = 10;

        /* Subsystem Variables */

        public static final double pivot_angle_tolerance = 1.5;

    }

    public static final class FlywheelConstants {

        /* CAN IDs */

        public static final int top_flywheel_motor_ID = 17, 
                                bottom_flywheel_motor_ID = 16; // Falcon 500

        /* Motor Parameters */

        public static final boolean flywheel_motors_clockwise_positive = false, 
                                    flywheel_motors_brake = false, 
                                    flywheel_motors_invert_sensor = false;
                
        public static final double flywheel_motors_max_continuous_current = 15,
                                   flywheel_motors_max_current = 25, 

                                   flywheel_motors_min_percent_output = 0.02, 
                                   flywheel_motors_max_percent_output = 0.95, 
                                   
                                   flywheel_motors_max_percent_output_per_second = 3, 
                                   
                                   flywheel_motors_gear_ratio = 1.0, 
                                   
                                   flywheel_motors_calibration_time = 0.25,

                                   flywheel_motors_free_current = 10;

        /* Subsystem Variables */

        public static final boolean flywheel_motors_opposite = false; // they should turn in the same direction

        public static final double flywheel_shooting_rpm = 4500, 
                                   flywheel_amp_rpm = 800,
                                   flywheel_intake_rpm = -400;

    }

    public static final class HangConstants {

        /* CAN IDs */

        public static final int left_hang_motor_ID = 14,
                                right_hang_motor_ID = 15; // Falcon 500

        /* Motor Parameters */

        public static final boolean hang_motors_clockwise_positive = false, // Literally does not matter
                                    hang_motors_brake = true, 
                                    hang_motors_invert_sensor = false;
                
        public static final double hang_motors_max_continuous_current = 15,
                                   hang_motors_max_current = 25, 

                                   hang_motors_min_percent_output = 0.02, 
                                   hang_motors_max_percent_output = 0.95, 
                                   
                                   hang_motors_max_percent_output_per_second = 3, 
                                   
                                   hang_motors_gear_ratio = 25.0, // doesn't really matter because its a linear relationship
                                   
                                   hang_motors_calibration_time = 0.25, 
                                   
                                   hang_motors_free_current = 8;

        /* Subsystem Variables */
        
        public static final boolean hang_motors_opposite = false; // they should turn in opposite directions

        public static final double hanging_position = 1800;

    }

}
