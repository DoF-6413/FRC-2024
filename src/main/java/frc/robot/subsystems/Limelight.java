package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.GeneralConstants.*;

public class Limelight { // Not technically a subsystem; everything should be static

    private Swerve swerve;

    // Grab a reference to the Limelights NetworkTable for use in all methods
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // This may look like a possible bug when on the Red Alliance but it is NOT.
    //
    // In 2024, most of the WPILib Ecosystem transitioned to a single-origin coordinate system.
    // In 2023, your coordinate system origin changed based on your alliance color.
    //
    // For 2024 and beyond, the origin of your coordinate system should always be the "blue" origin.
    // FRC teams should ALWAYS use botpose_wpiblue for pose-related functionality
    //
    // See: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
    private static Supplier<Double[]> limelightData = () -> limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, });

    private StructLogEntry<Pose2d> visionPoseLog;

    private StructPublisher<Pose2d> visionPoseNT;
    
    /**
     * Constructs a Limelight object associated with the given Swerve subsystem.
     *
     * This constructor initializes the Limelight and sets up any necessary communication with the Swerve drivebase to enable vision-based control.
     *
     * @param swerve The Swerve drivebase object.
     */
    public Limelight(Swerve swerve) {
        super();
        this.swerve = swerve;

        // Set the Limelight into Vision Processor mode using pipline 0 and turn off the LEDs
        setPipeline(0);
        limelightTable.getEntry("ledMode").setNumber(1);

        visionPoseNT = NetworkTableInstance.getDefault().getStructTopic("/Vision/Pose", Pose2d.struct).publish();
        var log = DataLogManager.getLog();
        visionPoseLog = StructLogEntry.create(log, "/Vision/Pose", Pose2d.struct);
    }

    public void periodic() {
      var data = limelightData.get();
      Translation2d translation = new Translation2d(data[0], data[1]);
      // z, roll, pitch
      Rotation2d yaw = Rotation2d.fromDegrees(data[5]);
      double latency = data[6];
      double tagCount = data[7];
      double averageTagArea = data[10];
      Pose2d pose = new Pose2d(translation, yaw);
      Pose2d swervePose = Swerve.getPose();

      ChassisSpeeds speeds = swerve.getRobotRelativeSpeeds();
      double poseDifference = swervePose.getTranslation().getDistance(pose.getTranslation());

      // If we are very close to the origin or there are no visible AprilTags log the info and stop
      if (translation.getNorm() < 0.1 || tagCount == 0) {
        log("Limelight State", "No Data");
        log("Limelight xyStds", 1000);
        log("Limelight degStds", 1000);
        log("Limelight Latency", latency);
      
        visionPoseLog.append(pose, (long) (Timer.getFPGATimestamp() - latency / 1000));
        visionPoseNT.set(pose);

        return;
      }

      double xyStds, degStds;

      if (tagCount >= 2) {
        // we see 2 tags, so we're super correct
        xyStds = 0.5;
        degStds = 6;
        log("Limelight State", "2 tag");
      } else if (averageTagArea > .4 && poseDifference < 0.5) {
        // we see 1 tag, but it's a big one
        xyStds = 1.0;
        degStds = 12;
        log("Limelight State", "1 big tag");
      } else if (averageTagArea > .05 && poseDifference < .3) {
        // we see 1 tag, but it's a small one and it's close to where we are
        xyStds = 2.0;
        degStds = 30;
        log("Limelight State", "1 small tag");
      } else {
        log("Limelight State", "Bad Data");
        log("Limelight xyStds", 1000);
        log("Limelight degStds", 1000);
        log("Limelight Latency", latency);

        visionPoseLog.append(pose, (long) (Timer.getFPGATimestamp() - latency / 1000));
        visionPoseNT.set(pose);

        return;
      }

      double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      xyStds *= (translationSpeed * 0.25 + 1);
      degStds *= (speeds.omegaRadiansPerSecond * 0.5 + 1);

      log("Limelight xyStds", xyStds);
      log("Limelight degStds", degStds);
      log("Limelight Latency", latency);
      
      visionPoseLog.append(pose, (long) (Timer.getFPGATimestamp() - latency / 1000));
      visionPoseNT.set(pose);

      Swerve.poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latency / 1000, VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds) * 3));
    }

    /**
     * Sets the Limelight pipeline to use based on the alliance color.
     *
     * If the robot is on the blue alliance, the pipeline is set to 0. Otherwise, it is set to 1.
     *
     * @param onBlueAlliance True if the robot is on the blue alliance, false otherwise.
     */
    public static void setPipline(boolean onBlueAlliance) {
        if (onBlueAlliance)
            setPipeline(0);
        else
            setPipeline(1);
    }

    /**
     * Sets the Limelight pipeline mode.
     *
     * This function sets the Limelight's pipeline mode to the specified number and make sure that
     * the Limelight is configured in Vision processor mode and not Driver Camera mode.
     *
     * @param number The Limelight pipeline to select.
     */
    public static void setPipeline(int number) {
        limelightTable.getEntry("camMode").setNumber(0); // Select Vision processor mode
        limelightTable.getEntry("pipeline").setNumber(number); // Select the pipeline to use
    }
}
