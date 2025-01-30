package frc.robot.subsystems.vision.apriltagvision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

// TODO tune all of these!!
public final class AprilTagVisionConstants {

  public static final boolean LEFT_CAM_ENABLED = true;
  public static final VisionConstants.CameraConstants LEFT_CAM_CONSTANTS =
      new VisionConstants.CameraConstants(
          "lefttagcam",
          new Transform3d(
              new Translation3d(0.3, -0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-14), degreesToRadians(0))),
          VisionConstants.CameraType.TELEPHOTO_OV9281,
          0.5);

  public static final boolean RIGHT_CAM_ENABLED = true;
  public static final VisionConstants.CameraConstants RIGHT_CAM_CONSTANTS =
      new VisionConstants.CameraConstants(
          "righttagcam",
          new Transform3d(
              new Translation3d(0.3, 0.3, 0.15),
              new Rotation3d(0, degreesToRadians(-14), degreesToRadians(0))),
          VisionConstants.CameraType.TELEPHOTO_OV9281,
          0.5);

  public static final DoubleSupplier[] TRANSLATION_EULER_MULTIPLIERS =
      new DoubleSupplier[] {
        new LoggedTunableNumber("AprilTagVision/EulerMultipliers/1Tag", 8),
        new LoggedTunableNumber("AprilTagVision/EulerMultipliers/2Tags", 10),
        new LoggedTunableNumber("AprilTagVision/EulerMultipliers/3Tags", 9)
      };
  public static final DoubleSupplier[] ROTATION_EULER_MULTIPLIERS =
      new DoubleSupplier[] {
        new LoggedTunableNumber("AprilTagVision/EulerMultipliers/1Tag", 20),
        new LoggedTunableNumber("AprilTagVision/EulerMultipliers/1Tag", 30),
        new LoggedTunableNumber("AprilTagVision/EulerMultipliers/1Tag", 25)
      };

  public static final double MAX_AMBIGUITY_CUTOFF = 0.05;
  public static final double MAX_Z_ERROR = 0.05;
}
