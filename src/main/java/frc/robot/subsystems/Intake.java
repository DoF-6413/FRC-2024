package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EasyCANSparkMax;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.GeneralConstants.*;

public class Intake extends SubsystemBase {

    private EasyCANSparkMax intake_motor;

    public Intake() {
        intake_motor = new EasyCANSparkMax(INTAKE_MOTOR_ID, INTAKE_MOTOR_TYPE, INTAKE_MOTOR_MAX_CONTINUOS_CURRENT, 
            INTAKE_MOTOR_MAX_CURRENT, INTAKE_MOTOR_CLOCKWISE_POSITIVE, INTAKE_MOTOR_BRAKE, INTAKE_MOTOR_MAX_PERCENT_OUTPUT_PER_SECOND);
        stop();
    }

    public void intake() {
        intake_motor.set(INTAKE_MOTOR_SPEED);
        log("Intake State", "Intaking");
    }

    public void retract() {
        intake_motor.set(-INTAKE_MOTOR_SPEED);
        log("Intake State", "Retracting");
    }

    public void stop() {
        intake_motor.set(0);
        log("Intake State", "Off");
    }

    public double getCurrent() {
        return intake_motor.getCurrent();
    }

    public boolean isFree() {
        return getCurrent() < INTAKE_MOTOR_FREE_CURRENT;
    }

    @Override
    public void periodic() {
        log("Intake Motor Current", getCurrent());
    }
}
