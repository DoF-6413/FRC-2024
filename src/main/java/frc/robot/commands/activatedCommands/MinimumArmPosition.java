package frc.robot.commands.activatedCommands;

import static frc.robot.Constants.TuningConstants.*;

import frc.robot.commands.subcommands.SetArmPosition;

import frc.robot.subsystems.Pivot;

public class MinimumArmPosition extends SetArmPosition {
    
    public MinimumArmPosition(Pivot pivot) {
        super(pivot, STARTING_ANGLE + PIVOT_GUARD_ANGLE);
    }

}