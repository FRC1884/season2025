package frc.robot.subsystems.vision;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.GlobalConstants.ROBOT;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

// TODO tune all of these!! Make sure that the robot-relative coords are correct - this will cause
// rapid pose oscillation
public final class AprilTagVisionConstants {
  public static final boolean LEFT_CAM_ENABLED = true;
  public static final VisionIO.CameraConstants LEFT_CAM_CONSTANTS =
      switch (ROBOT) {
        case DEVBOT -> new VisionIO.CameraConstants(
            "lefttagcam",
            new Transform3d(
                0.2752,
                0.2852,
                0.2375,
                new Rotation3d(0, degreesToRadians(-20), degreesToRadians(-20))),
            VisionIO.CameraType.OV9281);
        case COMPBOT, SIMBOT -> new VisionIO.CameraConstants(
            "lefttagcam",
            new Transform3d(
                0.3006,
                0.3056,
                0.245,
                new Rotation3d(0, degreesToRadians(-5), degreesToRadians(-20))),
            VisionIO.CameraType.OV9281);
      };

  public static final boolean RIGHT_CAM_ENABLED = false;
  public static final VisionIO.CameraConstants RIGHT_CAM_CONSTANTS =
      switch (ROBOT) {
        case DEVBOT -> new VisionIO.CameraConstants(
            "righttagcam",
            new Transform3d(
                0.2752,
                -0.2852,
                0.2375,
                new Rotation3d(0, degreesToRadians(-20), degreesToRadians(20))),
            VisionIO.CameraType.OV9281);
        case COMPBOT, SIMBOT -> new VisionIO.CameraConstants(
            "righttagcam",
            new Transform3d(
                0.3006,
                -0.3056,
                0.245,
                new Rotation3d(0, degreesToRadians(-5), degreesToRadians(20))),
            VisionIO.CameraType.OV9281);
      };

  public static final boolean BACK_CAM_ENABLED = true;
  public static final VisionIO.CameraConstants BACK_CAM_CONSTANTS =
      switch (ROBOT) {
        case DEVBOT -> new VisionIO.CameraConstants(
            "backtagcam",
            new Transform3d(
                -0.2927,
                0.3127,
                0.275,
                new Rotation3d(0, degreesToRadians(-14), degreesToRadians(180))),
            VisionIO.CameraType.OV9281);
        case COMPBOT, SIMBOT -> new VisionIO.CameraConstants(
            "backtagcam",
            new Transform3d(
                -0.3006,
                0.3056,
                0.245,
                new Rotation3d(0, degreesToRadians(-20), degreesToRadians(180))),
            VisionIO.CameraType.OV9281);
      };

  public static final DoubleSupplier TRANSLATION_EULER_MULTIPLIER =
      new LoggedTunableNumber("AprilTagVision/EulerMultipliers/Translation", 0.02);
  public static final DoubleSupplier ROTATION_EULER_MULTIPLIER =
      new LoggedTunableNumber("AprilTagVision/EulerMultipliers/Rotation", 0.06);

  public static final double MAX_AMBIGUITY_CUTOFF = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  public static final double kVisionStdDevX = 2;
  public static final double kVisionStdDevY = 2;
  public static final double kVisionStdDevTheta = 1;

  /**
   * Rough values for how much we trust this camera to produce reliable data on our target relative
   * to the other cameras. A lower value means we trust this camera more - for instance, if we're
   * more confident in its calibration than the other cameras. If any cameras are not explicitly
   * listed here, they will have a deafult ambiguity factor of 1.0.
   */
  public static final DoubleSupplier[] CAMERA_AMBIGUITY_FACTORS =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber(
            "AprilTagVision/" + LEFT_CAM_CONSTANTS.cameraName() + "CameraAmbiguityFactors", 1),
        new LoggedTunableNumber(
            "AprilTagVision/" + RIGHT_CAM_CONSTANTS.cameraName() + "CameraAmbiguityFactors", 1)
      };

  public static final double TAG_PRESENCE_WEIGHT = 10;
  public static final double DISTANCE_WEIGHT = 7;
  public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
  public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
      MatBuilder.fill(Nat.N3(), Nat.N1(), kVisionStdDevX, kVisionStdDevY, kVisionStdDevTheta);
}
