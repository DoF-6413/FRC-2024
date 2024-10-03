package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.autonomous.*;
import frc.robot.commands.activatedCommands.*;
import frc.robot.commands.defaultCommands.*;
import frc.robot.commands.subcommands.SetArmPosition;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.NamedCommands;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver_XBox = new XboxController(driver_usb_port);
    private final CommandXboxController operator = new CommandXboxController(operator_usb_port);

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver_XBox, XboxController.Button.kStart.value);

    private final Trigger robotCentric = new JoystickButton(driver_XBox, XboxController.Button.kY.value);

    /* Custom Triggers */

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Flywheel flywheel = new Flywheel();
    private final Pivot pivot = new Pivot();
    private final Hang hang = new Hang();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        if (relative_drive) {
            swerve.setDefaultCommand(
                new TeleopSwerveRelativeDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(xBoxTranslationAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxStrafeAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxRotationAxis), 
                    () -> false, 
                    () -> -driver_XBox.getPOV() + 1, 
                    () -> 1 - 0.75 * driver_XBox.getRawAxis(XboxController.Axis.kLeftTrigger.value), // what we multiply translation speed by; rotation speed is NOT affected
                    () -> (Variables.bypass_rotation || driver_XBox.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2)
                )
            );
        } else {
            swerve.setDefaultCommand(
                new TeleopSwerveAbsoluteDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(xBoxTranslationAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxStrafeAxis), 
                    () -> driver_XBox.getRawAxis(xBoxDirectionXAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxDirectionYAxis), 
                    () -> -driver_XBox.getPOV(), 
                    () -> (driver_XBox.getRawButton(XboxController.Button.kRightBumper.value) ? 0.2 : 0) -
                          (driver_XBox.getRawButton(XboxController.Button.kLeftBumper.value) ? 0.2 : 0), 
                    () -> (driver_XBox.getRawButton(xBoxSlowButtonOne) || driver_XBox.getRawButton(xBoxSlowButtonTwo)), 
                    () -> (Variables.bypass_rotation || driver_XBox.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2)
                )
            );
        } 

        pivot.setDefaultCommand(
            new TeleopPivot(
                pivot, 
                () -> operator.getLeftY()
            )
        );

        hang.setDefaultCommand(
            new TeleopHang(
                hang, 
                operator::getRightY, // no negative sign intentionally
                operator.povLeft(),
                operator.povRight()
            )
        );

        operator.povUp().whileTrue(Commands.run(() -> ShootingMath.v += 0.01));
        operator.povDown().whileTrue(Commands.run(() -> ShootingMath.v -= 0.01));

        conveyor.setDefaultCommand(new DormantConveyor(conveyor));
        flywheel.setDefaultCommand(new DormantFlywheel(flywheel));
        intake.setDefaultCommand(new DormantIntake(intake));

        configureButtonBindings();
        configureNamedCommands();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Triggers */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro(0)));
        
        // makeX.onTrue(new InstantCommand(() -> swerve.makeX())); button conflict :(

        /* Operator Triggers */
        operator.a().whileTrue(Commands.startEnd(flywheel::outtake, flywheel::stop, flywheel));
        operator.y().whileTrue(Commands.startEnd(flywheel::amp, flywheel::stop, flywheel));
        operator.b().toggleOnTrue(new RetractConveyor(conveyor)); // also ends all other commands requiring flywheel

        operator.x().toggleOnTrue(new SetArmPosition(pivot, starting_angle));
        operator.start().toggleOnTrue(new ResetHangCommand(hang));

        // shooting
        operator.leftTrigger(0.8).toggleOnTrue(ShootingCommands.amp(pivot));
        operator.rightTrigger(0.8).whileTrue(Commands.startEnd(() -> { Variables.bypass_angling = true; }, () -> { Variables.bypass_angling = false; }));

        // intake
        operator.leftBumper().toggleOnTrue(new IntakeCommand(pivot, intake, conveyor).handleInterrupt(() -> new RetractConveyor(conveyor).schedule()));
        operator.rightBumper().toggleOnTrue(new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(max_flywheel_acceleration_time), 
                    Commands.waitUntil(flywheel::isAtSpeed)
                ),
                new InstantCommand(conveyor::outtake, conveyor), 
                new WaitCommand(3),
                new InstantCommand(conveyor::stop, conveyor)
            ).handleInterrupt(() -> { conveyor.stop(); })
        );
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("AutonomousOuttake", new AutonomousOuttake(swerve, pivot, conveyor, flywheel));
        NamedCommands.registerCommand("AutonomousIntake", new AutonomousIntake(pivot, intake, conveyor));

        AutonomousCommands.configureNamedCommands(swerve, pivot, intake, conveyor, flywheel);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new ParallelCommandGroup(
            Autonomous.getAutonomousCommand(),
            new ResetHangCommand(hang)
        ).handleInterrupt(() -> Variables.bypass_rotation = false);
    }

    public void teleopInit() {
        swerve.teleopInit();
        pivot.brake();
    }

    public Swerve getSwerve() {
        return swerve;
    }
}
