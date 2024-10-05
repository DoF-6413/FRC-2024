package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIGETalon;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.PivotConstants.*;

public class Pivot extends SubsystemBase {
    
    private PIGETalon pivot_motor;
    private DoubleLogEntry targetAngleLog, currentAngleLog;

    public Pivot() {
        pivot_motor = new PIGETalon(PIVOT_MOTOR_ID, PIVOT_MOTOR_MAX_CONTINOUS_CURRENT, PIVOT_MOTOR_MAX_CURRENT, 
            PIVOT_MOTOR_BRAKE, PIVOT_MOTOR_CLOCKWISE_POSITIVE, STARTING_ANGLE, STARTING_ANGLE, AMP_ANGLE, PIVOT_MOTOR_MIN_PERCENT_OUTPUT, 
            PIVOT_MOTOR_MAX_PERCENT_OUTPUT, PIVOT_MOTOR_MIN_PERCENT_OUTPUT_PER_SECOND, PIVOT_MOTOR_GEAR_RATIO, 
            PIVOT_MOTOR_INVERT_SENSOR, PIVOT_MOTOR_CALIBRATION_TIME, PIVOT_P, PIVOT_I, PIVOT_G, PIVOT_E, PIVOT_ZERO
        );

        var log = DataLogManager.getLog();

        targetAngleLog = new DoubleLogEntry(log, "Pivot/TargetAngle");
        currentAngleLog = new DoubleLogEntry(log, "Pivot/CurrentAngle");
    }

    public void brake() {
        pivot_motor.brake();
    }

    public void move(double power) {
        pivot_motor.power(power);
    }
    
    public void pidPower() {
        pivot_motor.pidPower();
    }

    public void resetTarget() {
        pivot_motor.resetTarget();
    }

    public void setTarget(double angle) {
        pivot_motor.setTarget(angle);
    }

    public double getTargetAngle() {
        return pivot_motor.getTarget();
    }
    
    public double getPosition() {
        return pivot_motor.getEncoderPosition();
    }
    
    public double getCurrentError() {
        return getTargetAngle() - getPosition();
    }

    public boolean pidCloseEnough() {
        return Math.abs(getCurrentError()) < PIVOT_ANGLE_TOLERANCE;
    }

    public boolean isFree() {
        return pivot_motor.getCurrent() < PIVOT_MOTOR_FREE_CURRENT;
    }

    @Override
    public void periodic() {
        log("Pivot Current", pivot_motor.getCurrent());
        log("Pivot Motor Get", pivot_motor.get());
        log("Pivot Motor Sine", Math.sin((PIVOT_ZERO - pivot_motor.getEncoderPosition()) * Math.PI / 180));
        log("Possibly kG", pivot_motor.get() / Math.sin((PIVOT_ZERO - pivot_motor.getEncoderPosition()) * Math.PI / 180));
        log("Current Pivot Angle", pivot_motor.getEncoderPosition());
        log("Target Pivot Angle", pivot_motor.getTarget());
        log("Pivot At Target", pidCloseEnough() ? "yes" : "no");

        targetAngleLog.append(pivot_motor.getTarget());
        currentAngleLog.append(pivot_motor.getEncoderPosition());
    }
}
