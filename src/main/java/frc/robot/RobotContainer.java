package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.autonomous.*;
import frc.robot.commands.activatedCommands.*;
import frc.robot.commands.defaultCommands.*;
import frc.robot.commands.subcommands.SetArmPosition;
// import frc.robot.commands.utilCommands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.util.PathPlannerLogging;

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
    private final XboxController driver_XBox = new XboxController(DRIVER_USB_PORT);
    private final CommandXboxController operator = new CommandXboxController(OPERATOR_USB_PORT);
    private final Joystick driver_TFlightHotasOne = new Joystick(JOYSTICK_USB_PORT);

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver_XBox, XboxController.Button.kStart.value).or(new JoystickButton(driver_TFlightHotasOne, JOYSTICK_ZERO_GYRO_BUTTON));
    // private final Trigger makeX = new JoystickButton(driver_XBox, XBOX_MAKE_X_BUTTON).or(new JoystickButton(driver_TFlightHotasOne, JOYSTICK_MAKE_X_BUTTON));

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

        if (T_FLIGHT_HOT_AS_ONE_DRIVE) {
            swerve.setDefaultCommand(
                new TeleopSwerveRelativeDirecting(
                    swerve, 
                    () -> -driver_TFlightHotasOne.getRawAxis(JOYSTICK_TRANSLATION_AXIS)-driver_XBox.getRawAxis(XBOX_TRANSLATION_AXIS), 
                    () -> -driver_TFlightHotasOne.getRawAxis(JOYSTICK_STRAFE_AXIS)-driver_XBox.getRawAxis(XBOX_STRAFE_AXIS), 
                    () -> -driver_TFlightHotasOne.getRawAxis(JOYSTICK_ROTATION_AXIS)-driver_XBox.getRawAxis(XBOX_ROTATION_AXIS), 
                    () -> false, 
                    () -> -driver_TFlightHotasOne.getPOV()-driver_XBox.getPOV() + 1, 
                    () -> {
                        if (driver_TFlightHotasOne.getRawButton(JOYSTICK_SLOW_BUTTON)) {
                            return TELEOP_SWERVE_SLOW_FACTOR; // also sets direct angle I guess?
                        } else {
                            return 1 - 0.75 * driver_XBox.getRawAxis(XboxController.Axis.kLeftTrigger.value);
                        }
                    }, // what we multiply translation speed by; rotation speed is NOT affected
                    () -> (Variables.bypass_rotation || driver_TFlightHotasOne.getRawButton(JOYSTICK_DIRECT_ANGLE_BUTTON) || driver_XBox.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2)
                )
            );
        } else if (FANCY_DRIVE) {
            swerve.setDefaultCommand(
                new TeleopSwerveAbsoluteDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(XBOX_TRANSLATION_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_STRAFE_AXIS), 
                    () -> driver_XBox.getRawAxis(XBOX_DIRECTION_X_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_DIRECTION_Y_AXIS), 
                    () -> -driver_XBox.getPOV(), 
                    () -> (driver_XBox.getRawButton(XboxController.Button.kRightBumper.value) ? 0.2 : 0) - (driver_XBox.getRawButton(XboxController.Button.kLeftBumper.value) ? 0.2 : 0), 
                    () -> (driver_XBox.getRawButton(XBOX_SLOW_BUTTON_ONE) || driver_XBox.getRawButton(XBOX_SLOW_BUTTON_TWO)), 
                    () -> (Variables.bypass_rotation || driver_XBox.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2)
                )
            );
        } else {
            swerve.setDefaultCommand(
                new TeleopSwerveRelativeDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(XBOX_TRANSLATION_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_STRAFE_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_ROTATION_AXIS), 
                    () -> false, 
                    () -> -driver_XBox.getPOV(), 
                    () -> 1 - 0.75 * driver_XBox.getRawAxis(XboxController.Axis.kLeftTrigger.value), // what we multiply translation speed by; rotation speed is NOT affected
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

        // new JoystickButton(driver_TFlightHotasOne, JOYSTICK_DRIVE_TO_AMP_BUTTON).toggleOnTrue(TargetDriveCommands.driveToAmp(swerve));

        /* Operator Triggers */
        operator.a().whileTrue(Commands.startEnd(flywheel::outtake, flywheel::stop, flywheel));
        operator.y().whileTrue(Commands.startEnd(flywheel::amp, flywheel::stop, flywheel));
        operator.b().toggleOnTrue(new RetractConveyor(conveyor)); // also ends all other commands requiring flywheel

        operator.x().toggleOnTrue(new SetArmPosition(pivot, STARTING_ANGLE));
        operator.start().toggleOnTrue(new ResetHangCommand(hang));

        // shooting
        operator.leftTrigger(0.8).toggleOnTrue(ShootingCommands.amp(pivot));
        operator.rightTrigger(0.8).whileTrue(Commands.startEnd(() -> { Variables.bypass_angling = true; }, () -> { Variables.bypass_angling = false; }));

        // intake
        operator.leftBumper().toggleOnTrue(new IntakeCommand(pivot, intake, conveyor).handleInterrupt(() -> new RetractConveyor(conveyor).schedule()));
        operator.rightBumper().toggleOnTrue(new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(MAX_FLYWHEEL_ACCELERATION_TIME), 
                    Commands.waitUntil(flywheel::isAtSpeed)
                ),
                new InstantCommand(conveyor::outtake, conveyor), 
                new WaitCommand(3),
                new InstantCommand(conveyor::stop, conveyor)
            ).handleInterrupt(() -> { conveyor.stop(); })
        );

        new JoystickButton(driver_TFlightHotasOne, 5).toggleOnTrue(new SequentialCommandGroup(
            new InstantCommand(() -> intake.retract()), 
            new WaitCommand(3.0), 
            new InstantCommand(() -> intake.stop())
        ).handleInterrupt(intake::stop));
        new JoystickButton(driver_TFlightHotasOne, 6).toggleOnTrue(new SequentialCommandGroup(
            new InstantCommand(() -> intake.intake()), 
            new WaitCommand(3.0), 
            new InstantCommand(() -> intake.stop())
        ).handleInterrupt(intake::stop));
        new JoystickButton(driver_TFlightHotasOne, 8).toggleOnTrue(new InstantCommand(() -> intake.stop()));

        // right bumper auto pivots, hotas 2 auto directs

        /* Custom Triggers */

        new JoystickButton(driver_TFlightHotasOne, 14).and(() -> !DriverStation.isFMSAttached()).toggleOnTrue(new InstantCommand(() -> flywheel.cleaning()));

        new JoystickButton(driver_TFlightHotasOne, 13).or(new JoystickButton(driver_XBox, XboxController.Button.kBack.value)).toggleOnTrue(new InstantCommand(() -> swerve.overrideOdometry()));
        // new JoystickButton(driver_TFlightHotasOne, 14).or(new JoystickButton(driver_XBox, XboxController.Button.kBack.value)).toggleOnTrue(new InstantCommand(() -> Variables.bypass_angling = !Variables.bypass_angling));
        new JoystickButton(driver_TFlightHotasOne, 15).or(new JoystickButton(driver_XBox, XboxController.Button.kY.value)).toggleOnTrue(new InstantCommand(() -> Variables.bypass_rotation = !Variables.bypass_rotation));        
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
