package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final XboxController driver_XBox = new XboxController(DRIVER_USB_PORT);
    private final CommandXboxController operator = new CommandXboxController(OPERATOR_USB_PORT);

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver_XBox, XboxController.Button.kStart.value);
    private final Trigger robotCentric = new JoystickButton(driver_XBox, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Flywheel flywheel = new Flywheel();
    private final Pivot pivot = new Pivot();
    private final Hang hang = new Hang();

    // Create SmartDashboard chooser for autonomous routines
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */

    public RobotContainer() {

        if (RELATIVE_DRIVE) {
            swerve.setDefaultCommand(
                new TeleopSwerveRelativeDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(XBOX_TRANSLATION_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_STRAFE_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_ROTATION_AXIS), 
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
                    () -> -driver_XBox.getRawAxis(XBOX_TRANSLATION_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_STRAFE_AXIS), 
                    () -> driver_XBox.getRawAxis(XBOX_DIRECTION_X_AXIS), 
                    () -> -driver_XBox.getRawAxis(XBOX_DIRECTION_Y_AXIS), 
                    () -> -driver_XBox.getPOV(), 
                    () -> (driver_XBox.getRawButton(XboxController.Button.kRightBumper.value) ? 0.2 : 0) -
                          (driver_XBox.getRawButton(XboxController.Button.kLeftBumper.value) ? 0.2 : 0), 
                    () -> (driver_XBox.getRawButton(XBOX_SLOW_BUTTON_ONE) || driver_XBox.getRawButton(XBOX_SLOW_BUTTON_TWO)), 
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
        
        /* Operator Triggers */
        operator.a().whileTrue(Commands.startEnd(flywheel::outtake, flywheel::stop, flywheel));
        operator.y().whileTrue(Commands.startEnd(flywheel::amp, flywheel::stop, flywheel));
        operator.b().toggleOnTrue(new RetractConveyor(conveyor)); // also ends all other commands requiring flywheel
        operator.x().toggleOnTrue(new SetArmPosition(pivot, STARTING_ANGLE));
        operator.start().toggleOnTrue(new ResetHangCommand(hang));

        // shooting
        operator.leftTrigger(0.8).toggleOnTrue(ShootingCommands.amp(pivot));
        operator.rightTrigger(0.8).whileTrue(Commands.startEnd(
              () -> { Variables.bypass_angling = true; },
              () -> { Variables.bypass_angling = false; })
        );

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

        // Setup SmartDashboard options for Autonomous selection
        m_autoChooser.setDefaultOption("Do Nothing", new ParallelCommandGroup(
            new InstantCommand(() -> swerve.brake()),
            new InstantCommand(() -> intake.stop()),
            new InstantCommand(() -> conveyor.stop())));

        // TODO: Add new Autos with lines like: m_autoChooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));

        SmartDashboard.putData(m_autoChooser);
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
        return m_autoChooser.getSelected();
    }

    public void teleopInit() {
        swerve.teleopInit();
        pivot.brake();
    }
}
