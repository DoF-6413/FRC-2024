package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIETalon;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.HangConstants.*;

public class Hang extends SubsystemBase {

    private PIETalon left_hang_motor;
    private PIETalon right_hang_motor;

    public Hang() {

        left_hang_motor = new PIETalon(LEFT_HANG_MOTOR_ID, HANG_MOTORS_MAX_CONTINOUS_CURRENT, HANG_MOTORS_MAX_CURRENT, 
            HANG_MOTORS_BRAKE, HANG_MOTORS_CLOCKWISE_POSITIVE, 0, -HANGING_POSITION, 0, HANG_MOTORS_MIN_PERCENT_OUTPUT, 
            HANG_MOTORS_MAX_PERCENT_OUTPUT, HANG_MOTORS_MIN_PERCENT_OUTPUT_PER_SECOND, HANG_MOTORS_GEAR_RATIO, 
            HANG_MOTORS_INVERT_SENSOR, HANG_MOTORS_CALIBRATION_TIME, HANG_P, HANG_I, HANG_E
        );
        
        right_hang_motor = new PIETalon(RIGHT_HANG_MOTOR_ID, HANG_MOTORS_MAX_CONTINOUS_CURRENT, HANG_MOTORS_MAX_CURRENT, 
            HANG_MOTORS_BRAKE, HANG_MOTORS_CLOCKWISE_POSITIVE ^ HANG_MOTORS_OPPOSITE, 0, -HANGING_POSITION, 0, HANG_MOTORS_MIN_PERCENT_OUTPUT, 
            HANG_MOTORS_MAX_PERCENT_OUTPUT, HANG_MOTORS_MIN_PERCENT_OUTPUT_PER_SECOND, HANG_MOTORS_GEAR_RATIO, 
            HANG_MOTORS_INVERT_SENSOR, HANG_MOTORS_CALIBRATION_TIME, HANG_P, HANG_I, HANG_E
        ); // invert and opposite: TT -> F, TF -> T, FT -> T, FF -> F; ^ is XOR

        left_hang_motor.disableLimiting();
        right_hang_motor.disableLimiting();

    }

    public void moveLeft(double power) {
        left_hang_motor.power(power);
    }

    public void moveRight(double power) {
        right_hang_motor.power(power);
    }

    public void moveVoltagePercent(double voltage) {
        moveRightVoltagePercent(voltage);
        moveLeftVoltagePercent(voltage);
    }

    public void moveRightVoltagePercent(double voltage) {
        if (voltage == 0) {
            right_hang_motor.brake();
        } else {
            right_hang_motor.setVoltagePercent(voltage);
        }
    }

    public void moveLeftVoltagePercent(double voltage) {
        if (voltage == 0) {
            left_hang_motor.brake();
        } else {
            left_hang_motor.setVoltagePercent(voltage);
        }
    }

    public boolean isLeftStuck() {
        return Math.abs(left_hang_motor.getVelocity()) < 0.4;
    }

    public boolean isRightStuck() {
        return Math.abs(right_hang_motor.getVelocity()) < 0.4;
    }

    public void disableLimiting() {
        right_hang_motor.disableLimiting();
        left_hang_motor.disableLimiting();
    }

    public void setCoastModes() {
        right_hang_motor.setBrake(false);
        left_hang_motor.setBrake(false);
    }

    public void setBrakeModes() {
        right_hang_motor.setBrake(true);
        left_hang_motor.setBrake(true);
    }

    public void relax() {
        left_hang_motor.setTarget(0);
        right_hang_motor.setTarget(0);
        log("Hang State", "Relaxed");
    }

    public void hang() {
        left_hang_motor.setTarget(0);
        right_hang_motor.setTarget(0);
        log("Hang State", "Hanging");
    }

    public void resetEncoders() {
        left_hang_motor.resetEncoder();
        right_hang_motor.resetEncoder();
    }

    public void enableLimiting() {
        left_hang_motor.enableLimiting(-HANGING_POSITION, 0);
        right_hang_motor.enableLimiting(-HANGING_POSITION, 0);
    }

    public boolean isFree() {
        return left_hang_motor.getCurrent() + right_hang_motor.getCurrent() < 2 * HANG_MOTORS_FREE_CURRENT;
    }

    @Override
    public void periodic() {
        log("Left Hang Motor Position", left_hang_motor.getEncoderPosition());
        log("Right Hang Motor Position", right_hang_motor.getEncoderPosition());
        
        log("Left Hang Motor Current", left_hang_motor.getCurrent());
        log("Right Hang Motor Current", right_hang_motor.getCurrent());

        log("Left Hang Motor Velocity", left_hang_motor.getVelocity());
        log("Right Hang Motor Velocity", right_hang_motor.getVelocity());
    }
}
