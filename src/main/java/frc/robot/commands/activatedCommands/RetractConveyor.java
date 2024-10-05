package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.TuningConstants.*;

public class RetractConveyor extends SequentialCommandGroup {

    public RetractConveyor(Conveyor conveyor) {
        addRequirements(conveyor);

        addCommands(
            new InstantCommand(() -> conveyor.retract()), 
            new WaitCommand(RETRACT_CONVEYOR_TIME).handleInterrupt(() -> conveyor.stop()), 
            new InstantCommand(() -> conveyor.stop())
        );
    }

}