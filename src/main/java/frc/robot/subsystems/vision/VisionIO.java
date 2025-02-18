package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /**
   * An enum containing every type of camera we might have on our robot, and thresholds of distances
   * to tags at which we think they will provide meaningful data on a target.
   */
  enum CameraType {
    OV2311(7.5),
    OV9281(2.5),
    LIMELIGHT(5.0),
    LIMELIGHT_3G(12.0),
    A1920(8.0),
    TELEPHOTO_OV2311(9.0),
    TELEPHOTO_OV9281(9.0),
    TELEPHOTO_A1920(10.0),
    TELEPHOTO_LIMELIGHT(7.5),
    TELEPHOTO_LIMELIGHT_3G(15.0),
    UNKNOWN(0.0);

    public final double noisyDistance;

    CameraType(double noisyDistance) {
      this.noisyDistance = noisyDistance;
    }
  }

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default CameraConstants getCameraConstants() {
    return new CameraConstants("UNKNOWN", new Transform3d(), CameraType.UNKNOWN);
  }

  /**
   * Represents the angle to a tag, not used for pose estimation.
   *
   * @param tx the horizontal angle (yaw) to the target, in degrees. Positive-right, center-zero.
   * @param ty the vertical angle (pitch) to the target, in degrees. Positive-down, center-zero.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * Represents a robot pose sample used for pose estimation.
   *
   * @param timestamp the timestamp of the estimate, in seconds.
   * @param pose the estimated pose, in meters.
   * @param ambiguity the ambiguity factor of the pose estimate.
   * @param tagCount the total number of tags seen.
   * @param averageTagDistance the average distance to all tags in frame, in meters.
   */
  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  /**
   * A data class representing all the information we need on a camera to get us from it being
   * connected and streaming, to a filtered pose estimate we can fuse with other cameras' estimates
   * and the drivetrain's odometry.
   *
   * @param cameraName the NetworkTables name for this camera. This is what will help us identify
   *     each camera on AdvantageScope and Elastic, to help with debugging.
   * @param robotToCamera the transformation representing the camera's position relative to the
   *     center of the robot. Translations are in meters.
   * @param cameraType the type of hardware camera we are using. See {@link CameraType}.
   */
  public static record CameraConstants(
      String cameraName, Transform3d robotToCamera, CameraType cameraType) {}
}
