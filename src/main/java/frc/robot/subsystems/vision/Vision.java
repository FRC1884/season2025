package frc.robot.subsystems.vision;

import static frc.robot.GlobalConstants.FieldMap.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.MAX_AMBIGUITY_CUTOFF;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.MAX_Z_ERROR;
import static frc.robot.subsystems.vision.AprilTagVisionHelpers.generateDynamicStdDevs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  /**
   * Creates a Vision system.
   *
   * @param consumer an object that processes the vision pose estimate (should be the drivetrain)
   * @param io the collection of {@link VisionIO}s instances that represent the cameras in the
   *     system.
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera \"" + io[i].getCameraConstants().cameraName() + "\" is disconnected.",
              Alert.AlertType.kWarning);
    }
  }

  /**
   * Returns the yaw (horizontal angle) to the best detected AprilTag if available. If no tags are
   * detected, an empty {@link Optional} is returned.
   *
   * @param cameraIndex The index of the camera to retrieve yaw data from.
   * @return An {@link Optional} containing the yaw as a {@link Rotation2d}, or empty if no tag is
   *     detected.
   */
  public Optional<Rotation2d> getTargetX(int cameraIndex) {
    return inputs[cameraIndex].tagIds.length == 0
        ? Optional.empty()
        : Optional.of(inputs[cameraIndex].latestTargetObservation.tx());
  }

  /**
   * Returns the pitch (vertical angle) to the best detected AprilTag if available. If no tags are
   * detected, an empty {@link Optional} is returned.
   *
   * @param cameraIndex The index of the camera to retrieve pitch data from.
   * @return An {@link Optional} containing the pitch as a {@link Rotation2d}, or empty if no tag is
   *     detected.
   */
  public Optional<Rotation2d> getTargetY(int cameraIndex) {
    return inputs[cameraIndex].tagIds.length == 0
        ? Optional.empty()
        : Optional.of(inputs[cameraIndex].latestTargetObservation.ty());
  }

  /**
   * Updates vision inputs and logs relevant pose data. Filters out invalid observations and sends
   * valid ones to the pose consumer for fused robot localization.
   */
  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/" + io[i].getCameraConstants().cameraName(), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update alert status for disconnected cameras
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > MAX_AMBIGUITY_CUTOFF) // Cannot be high ambiguity on single tag
                || Math.abs(observation.pose().getZ())
                    > MAX_Z_ERROR // Must have realistic Z coordinate
                || observation.averageTagDistance()
                    > io[cameraIndex]
                        .getCameraConstants()
                        .cameraType()
                        .noisyDistance // Must be reliably detectable by this camera
                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            generateDynamicStdDevs(observation, io[cameraIndex].getCameraConstants().cameraType()));
      }

      // Log camera data
      Logger.recordOutput(
          "AprilTagVision/" + io[cameraIndex].getCameraConstants().cameraName() + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "AprilTagVision/" + io[cameraIndex].getCameraConstants().cameraName() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "AprilTagVision/"
              + io[cameraIndex].getCameraConstants().cameraName()
              + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "AprilTagVision/"
              + io[cameraIndex].getCameraConstants().cameraName()
              + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      Logger.recordOutput(
          "AprilTagVision/" + io[cameraIndex].getCameraConstants().cameraName() + "/CameraPose",
          io[cameraIndex].getCameraConstants().robotToCamera());
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "AprilTagVision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPoses",
        allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  /** Functional interface defining a consumer that processes vision-based pose estimates. */
  @FunctionalInterface
  public interface VisionConsumer {
    /**
     * Accepts a vision pose estimate for processing.
     *
     * @param visionRobotPoseMeters The estimated robot pose, in meters.
     * @param timestampSeconds The timestamp of the observation for latency compensation, in
     *     seconds.
     * @param visionMeasurementStdDevs The standard deviations of the measurement.
     */
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
