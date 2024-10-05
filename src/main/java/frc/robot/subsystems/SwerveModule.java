package frc.robot.subsystems;

import static frc.robot.Constants.TuningConstants.USE_CANCODERS;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d ANGLE_OFFSET;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.BaseFalconSwerveConstants.DRIVE_KS, Constants.BaseFalconSwerveConstants.DRIVE_KV, Constants.BaseFalconSwerveConstants.DRIVE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.ANGLE_OFFSET = moduleConstants.ANGLE_OFFSET;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.CANCODER_ID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
        // Timer.delay(1.0);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.ANGLE_MOTOR_ID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        Timer.delay(1.0);

        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.DRIVE_MOTOR_ID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.BaseFalconSwerveConstants.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.BaseFalconSwerveConstants.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - ANGLE_OFFSET.getRotations();
        mAngleMotor.setPosition(USE_CANCODERS ? absolutePosition : -0.25);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.BaseFalconSwerveConstants.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModuleState getCanState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.BaseFalconSwerveConstants.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(getCANcoder().getRotations() - ANGLE_OFFSET.getRotations())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.BaseFalconSwerveConstants.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public double[] getCurrents() {
        return new double[] {
            mAngleMotor.getStatorCurrent().getValueAsDouble(), // stator instead of torque because we
            mDriveMotor.getStatorCurrent().getValueAsDouble()  // only care about absolute value
        };
    }

    public double getPositionRads() {
      return Units.rotationsToRadians(mDriveMotor.getPosition().getValue());
    }

    public void driveVoltage(double volts) {
        mDriveMotor.setVoltage(volts);
    }

    public double getVelocity() {
        return mDriveMotor.getVelocity().getValueAsDouble();
    }

    public double getVoltage() {
        return mDriveMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getDrivePosition() {
        return mDriveMotor.getPosition().getValueAsDouble();
    }
}
