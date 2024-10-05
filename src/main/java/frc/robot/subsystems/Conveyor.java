package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EasyCANSparkMax;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.ConveyorConstants.*;

public class Conveyor extends SubsystemBase {
    private EasyCANSparkMax conveyor_motor;

    public Conveyor() {
        conveyor_motor = new EasyCANSparkMax(CONVEYOR_MOTOR_ID, CONVEYOR_MOTOR_TYPE, CONVEYOR_MOTOR_MAX_CONTINOUS_CURRENT, 
            CONVEYOR_MOTOR_MAX_CURRENT, CONVEYOR_MOTOR_CLOCKWISE_POSITIVE, CONVEYOR_MOTOR_BRAKE, CONVEYOR_MOTOR_MAX_PERSENT_OUTPUT_PER_SECOND);
    }

    public void intake() {
        conveyor_motor.set(CONVEYOR_INTAKE_SPEED);
        log("Conveyor State", "Intaking");
    }

    public void outtake() {
        conveyor_motor.set(CONVEYOR_OUTTAKE_SPEED);
        log("Conveyor State", "Outtaking");
    }

    public void retract() {
        conveyor_motor.set(CONVEYOR_RETRACT_SPEED);
        log("Conveyor State", "Retracting");
    }

    public void stop() {
        conveyor_motor.set(0);
        log("Conveyor State", "Off");
    }

    public double getCurrent() {
        return conveyor_motor.getCurrent();
    }

    public boolean isFree() {
        return getCurrent() < CONVEYOR_MOTOR_FREE_CURRENT;
    }

    @Override
    public void periodic() {
        log("Conveyor Motor Current", getCurrent());
        log("Conveyor Power", conveyor_motor.getSpeed());
    }
}
