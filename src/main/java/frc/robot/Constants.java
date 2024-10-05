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

        public static final double CANCODER_0_ZERO = -47.81, // Front Left
                                   CANCODER_1_ZERO = -47.15, // Front Right
                                   CANCODER_2_ZERO = -128.5, // Back Left
                                   CANCODER_3_ZERO = -88; // Back Right
        
        public static final boolean USE_CANCODERS = true;
        
        /* Swerve Drive Constants */

        public static final double DRIVE_MOTOR_P = 0.05,
                                   DRIVE_STATIC_VOLTAGE = 0.32, 
                                   DRIVE_EQUILIBRIUM_VOLTAGE = 1.51, 
                                   DRIVE_ACCELERATION_VOLTAGE = 0.27, // SYSID values: KS, KV, KA; they are automatically divided by 12 later
                                   MAX_LINEAR_SPEED = 10, // feet per second; theoretical max is 13.5
                                   MAX_ANGULAR_SPEED = 360; // degrees per second; theoretical max is theoretical maximum is MAX_LINEAR_SPEED * 46.6436741705 which is roughly 629.6896013018

        /* PathPlanner PID Constants */

        public static final double AUTONOMOUS_MAX_LINEAR_SPEED = 10,
                                   AUTONOMOUS_RAMP_IP_TIME_LINEAR = 0.75, // in seconds to reach max 
                                   AUTONOMOUS_MAX_ANGULAR_SPEED = 400, 
                                   AUTONOMOUS_RAMP_UP_TIME_ANGULAR = 0.66,
                                   AUTONOMOUS_TRANSLATION_P_CONTROLLER = 2, 
                                   AUTONOMOUS_ANGLE_P_CONTROLLER = 4;
        
        /* Driving PID Constants */

        public static final double TELEOP_ANGLE_P = 0.165,
                                   TELEOP_ANGLE_I = 0,
                                   TELEOP_ANGLE_E = 1.35,
                                   TELEOP_TRANSLATION_P = 0.4, 
                                   TELEOP_TRANSLATION_I = 0,
                                   TELEOP_TRANSLATION_TOLERANCE = 1;
        
        public static final double turn_slow_ratio = 1; // because slowing down rotation and translation by the same factor is insane
                                    // ex. it its 4, then 0.6 translation ratio goes to a 0.9 turning ratio
        
        /* Pivot Constants */

        public static final double PIVOT_P = .015, 
                                   PIVOT_I = 0, 
                                   PIVOT_G = 0.018, 
                                   PIVOT_E = 1.05, 
                                   PIVOT_ZERO = 115;
        
        public static final double STARTING_ANGLE = -9.255644, // thanks to certain people, it's not -9.255644, // all in degrees
                                   INTAKE_ANGLE = 59.006106, 
                                   AMP_ANGLE = 125.388397, 

                                   MIN_OUTTAKE_ANGLE = 25, 
                                   PIVOT_GUARD_ANGLE = 0; // so we dont smack against the walls. goal is to get this to 0 soon
        
        /* Hang Constants */
        
        public static final double HANG_P = 0.0, // temporarily disabled
                                   HANG_I = 0.0, 
                                   HANG_E = 1.0;
        
        /* Intaking Constants */

        public static final double ACCELERATION_TIME = 1, 
                                   MIN_INTAKE_TIME = 8, 
                                   ADD_INTAKE_TIME = 0.8, 
                                   ADD_CONVEYOR_TIME = 0.2, 
                                   RETRACT_CONVEYOR_TIME = 0.15; // how much we intake after the note has passed through
        
        /* Outtaking Constants */

        public static final double MAX_FLYWHEEL_ACCELERATION_TIME = 4, 
                                   MIN_OUTTAKE_TIME = 1, 
                                   MIN_AMP_TIME = 2;

    }


    public static final class Buttons {

        /* D-pad */

        public static final int DPAD_UP = 0, 
                                DPAD_RIGHT = 90, 
                                DPAD_DOWN = 180, 
                                DPAD_LEFT = 270;

        /* Which USB port is used for the different controllers */

        public static final int DRIVER_USB_PORT = 2, 
                                OPERATOR_USB_PORT = 1;

        /* Driver Buttons */

        public static final int XBOX_TRANSLATION_AXIS = XboxController.Axis.kLeftY.value, 
                                XBOX_STRAFE_AXIS = XboxController.Axis.kLeftX.value,

                                XBOX_ROTATION_AXIS = XboxController.Axis.kRightX.value,

                                XBOX_DIRECTION_X_AXIS = XboxController.Axis.kRightX.value, 
                                XBOX_DIRECTION_Y_AXIS = XboxController.Axis.kRightY.value, 
                                XBOX_TURN_RIGHT_AXIS = XboxController.Axis.kRightTrigger.value, 
                                XBOX_TURN_LEFT_AXIS = XboxController.Axis.kLeftTrigger.value, 
                                
                                XBOX_SLOW_AXIS = XboxController.Axis.kRightTrigger.value, 
                                XBOX_SLOW_BUTTON_ONE = XboxController.Button.kRightBumper.value, 
                                XBOX_SLOW_BUTTON_TWO = XboxController.Button.kLeftBumper.value, 

                                XBOX_ROBOT_CENTRIC_BUTTON = XboxController.Button.kLeftBumper.value,

                                xBOX_ZERO_GYRO_BUTTON = XboxController.Button.kY.value, 
                                XBOX_MAKE_X_BUTTON = XboxController.Button.kX.value, 
                                XBOX_TERMINATE_COMMANDS_dRIVER_BUTTON = XboxController.Button.kBack.value,

                                JOYSTICK_TRANSLATION_AXIS = 1, 
                                JOYSTICK_STRAFE_AXIS = 0, 

                                JOYSTICK_ROTATION_AXIS = 5, 

                                JOYSTICK_SLOW_BUTTON = 1,

                                JOYSTICK_DIRECT_ANGLE_BUTTON = 2, 

                                JOYSTICK_ZERO_GYRO_BUTTON = 4, 
                                JOYSTICK_MAKE_X_BUTTON = 2, 
                                JOYSTICK_DRIVE_TO_AMP_BUTTON = 4; // if we push the slider forward it cancels commands

        /* Operator Buttons */
        public static final int TERMINATE_COMMANDS_OPERATOR_BUTTON = XboxController.Button.kBack.value;
        
    }


    public static final class GeneralConstants {

        public static final boolean RELATIVE_DRIVE = true;

        public static final double JOYSTICK_DEADZONE = 0.2; // The deadband value to use on any joystick

        public static final double AXIS_EXPONENT = 1.3;
        
        public static final double STARTING_YAW = 0; // shoot this might depend based on autonomous... we can make a new command xoxo

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
            axis_value = MathUtil.applyDeadband(axis_value, JOYSTICK_DEADZONE);

            // If the value is 0 we are done
            if (axis_value == 0)
                return 0;

            // Calculate the signed power.  If the value is negative, then we want
            // the returned value to also be negative.
            if (axis_value < 0)
                return 0 - Math.pow(0 - axis_value, AXIS_EXPONENT);

            // Otherwise we return the positive signed power
            return Math.pow(axis_value, AXIS_EXPONENT);
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
        public static final int PIGEON_ID = 0;

        public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75); // 28" width -> 22.75" track width
        public static final double WHEEL_BASE = Units.inchesToMeters(18.75); // 24" drivetrain length -> 18.75" wheel base
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.WHEEL_CIRCUMFERENCE;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.DRIVE_GEAR_RATIO;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.ANGLE_GEAR_RATIO;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.ANGLE_MOTOR_INVERT;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.DRIVE_MOTOR_INVERT;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.CANCODER_INVERT;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 30;
        public static final int ANGLE_CURRENT_THRESHOLD = 35;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 30;
        public static final int DRIVE_CURRENT_THRESHOLD = 35;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.ANGLE_KP;
        public static final double ANGLE_KI = CHOSEN_MODULE.ANGLE_KI;
        public static final double ANGLE_KD = CHOSEN_MODULE.ANGLE_KD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = TuningConstants.DRIVE_MOTOR_P; 
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = (TuningConstants.DRIVE_STATIC_VOLTAGE / 12.0);
        public static final double DRIVE_KV = (TuningConstants.DRIVE_EQUILIBRIUM_VOLTAGE / 12.0);
        public static final double DRIVE_KA = (TuningConstants.DRIVE_ACCELERATION_VOLTAGE / 12.0);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = TuningConstants.MAX_LINEAR_SPEED * 0.3048;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = TuningConstants.MAX_ANGULAR_SPEED * Math.PI / 180.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(TuningConstants.CANCODER_0_ZERO + 90);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 5;
            public static final int CANCODER_ID = 6;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(TuningConstants.CANCODER_1_ZERO + 90);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 9;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(TuningConstants.CANCODER_2_ZERO + 90);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 11; // bc they were messed up from the can id thing
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 12;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(TuningConstants.CANCODER_3_ZERO + 90);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = TuningConstants.AUTONOMOUS_MAX_LINEAR_SPEED * 0.3048;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND = TuningConstants.AUTONOMOUS_MAX_LINEAR_SPEED * 0.3048 / TuningConstants.AUTONOMOUS_RAMP_IP_TIME_LINEAR;
        public static final double MAX_ANGULAR_SPEED_RADIANTS_PER_SECOND = TuningConstants.AUTONOMOUS_MAX_ANGULAR_SPEED * Math.PI / 180.0;
        public static final double MAX_ANGULAR_SPEED_RADIANTS_PER_SECOND_SQUARED = TuningConstants.AUTONOMOUS_MAX_ANGULAR_SPEED * Math.PI / 180.0 / TuningConstants.AUTONOMOUS_RAMP_UP_TIME_ANGULAR;
    
        public static final double KP_X_CONTROLLER = TuningConstants.AUTONOMOUS_TRANSLATION_P_CONTROLLER;
        public static final double KP_Y_CONTROLLER = TuningConstants.AUTONOMOUS_TRANSLATION_P_CONTROLLER;
        public static final double KP_THETHA_CONTROLLER = TuningConstants.AUTONOMOUS_ANGLE_P_CONTROLLER;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints THETHA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANTS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANTS_PER_SECOND_SQUARED);
    }

    public static final class TeleopSwerveConstants {
        public static final double SWERVE_MIN_PID_ROTATION = 0.05,
                                   SWERVE_MAX_PID_ROTATION = 0.8, 
                                   SWERVE_CALIBRATION_TIME = 0.5, 
                                   SWERVE_MIN_MANUAL_TRANSLATION = 0.05, 
                                   SWERVE_MIN_MANUAL_ROTATION = 0.02,

                                   TELEOP_SWERVE_SLOW_FACTOR = 0.2, 
                                   
                                   VISION_TOLERANCE = 5, 
                                   
                                   TELEOP_ROTATIO_PERCENT = 0.75, 
                                   SWERVE_BUMPER_TURN_SENSIVILITY = 0.35; // ratio of teleop swerve rotation speed vs maximum swerve rotation speed
    }
    
    public static final class IntakeConstants {

        /* CAN IDs */

        public static final int INTAKE_MOTOR_ID = 20; // Neo
        
        /* Motor Parameters */

        public static final MotorType INTAKE_MOTOR_TYPE = MotorType.kBrushless;
        
        public static final boolean INTAKE_MOTOR_CLOCKWISE_POSITIVE = true, 
                                    INTAKE_MOTOR_BRAKE = true;
        
        public static final int INTAKE_MOTOR_MAX_CONTINUOS_CURRENT = 15;

        public static final double INTAKE_MOTOR_MAX_CURRENT = 25, 

                                   INTAKE_MOTOR_MAX_PERCENT_OUTPUT_PER_SECOND = 3, 

                                   INTAKE_MOTOR_FREE_CURRENT = 11;

        /* Subsystem Variables */

        public static final double INTAKE_MOTOR_SPEED = 1.0;
        
    }

    public static final class ConveyorConstants {

        /* CAN IDs */

        public static final int CONVEYOR_MOTOR_ID = 19; // Neo
        
        /* Motor Parameters */
        
        public static final MotorType CONVEYOR_MOTOR_TYPE = MotorType.kBrushless;

        public static final boolean CONVEYOR_MOTOR_CLOCKWISE_POSITIVE = true, 
                                    CONVEYOR_MOTOR_BRAKE = true;
        
        public static final int CONVEYOR_MOTOR_MAX_CONTINOUS_CURRENT = 10;

        public static final double CONVEYOR_MOTOR_MAX_CURRENT = 15, 
                                           
                                   CONVEYOR_MOTOR_MAX_PERSENT_OUTPUT_PER_SECOND = 3,

                                   CONVEYOR_MOTOR_FREE_CURRENT = 5;

        /* Subsystem Variables */

        public static final double CONVEYOR_INTAKE_SPEED = 0.7, 
                                   CONVEYOR_RETRACT_SPEED = -0.5, 
                                   CONVEYOR_OUTTAKE_SPEED = 0.8, 
                                   CONVEYOR_AMP_SPEED = 0.8;

    }

    public static final class PivotConstants {

        /* CAN IDs */

        public static final int PIVOT_MOTOR_ID = 13; // Kraken

        /* Motor Parameters */

        public static final boolean PIVOT_MOTOR_CLOCKWISE_POSITIVE = false, 
                                    PIVOT_MOTOR_BRAKE = false, 
                                    PIVOT_MOTOR_INVERT_SENSOR = false;
                
        public static final double PIVOT_MOTOR_MAX_CONTINOUS_CURRENT = 15, 
                                   PIVOT_MOTOR_MAX_CURRENT = 25, 

                                   PIVOT_MOTOR_MIN_PERCENT_OUTPUT = 0, 
                                   PIVOT_MOTOR_MAX_PERCENT_OUTPUT = 0.95, 
                                   
                                   PIVOT_MOTOR_MIN_PERCENT_OUTPUT_PER_SECOND = 10, 
                                   
                                   PIVOT_MOTOR_GEAR_RATIO = (5.0 / 1.0) * (9.0 / 1.0) * (34.0 / 10.0), 
                                   
                                   PIVOT_MOTOR_CALIBRATION_TIME = 0.2, 

                                   PIVOT_MOTOR_FREE_CURRENT = 10;

        /* Subsystem Variables */

        public static final double PIVOT_ANGLE_TOLERANCE = 1.5;

    }

    public static final class FlywheelConstants {

        /* CAN IDs */

        public static final int TOP_FLYWHEEEL_MOTOR_ID = 17, 
                                BOTTOM_FLYWHEEL_MOTOR_ID = 16; // Falcon 500

        /* Motor Parameters */

        public static final boolean FLYWHEEL_MOTORS_CLOCKWISE_POSITIVE = false, 
                                    FLYWHEEL_MOTORS_BRAKE = false, 
                                    FLYWHEEL_MOTORS_INVERT_SENSOR = false;
                
        public static final double FLYWHEEL_MOTORS_MAX_CONTINUOUS_CURRENT = 15,
                                   FLYWHEEL_MOTORS_MAX_CURRENT = 25, 

                                   FLYWHEEL_MOTORS_MIN_PERCENT_OUTPUT = 0.02, 
                                   FLYWHEEL_MOTORS_MAX_PERCENT_OUTPUT = 0.95, 
                                   
                                   FLYWHEEL_MOTORS_MIN_PERCENT_OUTPUT_PER_SECOND = 3, 
                                   
                                   FLYWHEEL_MOTORS_GEAR_RATIO = 1.0, 
                                   
                                   FLYWHEEL_MOTORS_CALIBRATION_TIME = 0.25,

                                   FLYWHEEL_MOTORS_FREE_CURRENT = 10;

        /* Subsystem Variables */

        public static final boolean FLYWHEEL_MOTORS_OPPOSITE = false; // they should turn in the same direction

        public static final double FLYWHEEL_SHOOTING_RPM = 4500, 
                                   FLYWHEEL_AMP_RPM = 800,
                                   FLYWHEEL_INTAKE_RPM = -400;

    }

    public static final class HangConstants {

        /* CAN IDs */

        public static final int LEFT_HANG_MOTOR_ID = 14,
                                RIGHT_HANG_MOTOR_ID = 15; // Falcon 500

        /* Motor Parameters */

        public static final boolean HANG_MOTORS_CLOCKWISE_POSITIVE = false, // Literally does not matter
                                    HANG_MOTORS_BRAKE = true, 
                                    HANG_MOTORS_INVERT_SENSOR = false;
                
        public static final double HANG_MOTORS_MAX_CONTINOUS_CURRENT = 15,
                                   HANG_MOTORS_MAX_CURRENT = 25, 

                                   HANG_MOTORS_MIN_PERCENT_OUTPUT = 0.02, 
                                   HANG_MOTORS_MAX_PERCENT_OUTPUT = 0.95, 
                                   
                                   HANG_MOTORS_MIN_PERCENT_OUTPUT_PER_SECOND = 3, 
                                   
                                   HANG_MOTORS_GEAR_RATIO = 25.0, // doesn't really matter because its a linear relationship
                                   
                                   HANG_MOTORS_CALIBRATION_TIME = 0.25, 
                                   
                                   HANG_MOTORS_FREE_CURRENT = 8;

        /* Subsystem Variables */
        
        public static final boolean HANG_MOTORS_OPPOSITE = false; // they should turn in opposite directions

        public static final double HANGING_POSITION = 1800;

    }

}
