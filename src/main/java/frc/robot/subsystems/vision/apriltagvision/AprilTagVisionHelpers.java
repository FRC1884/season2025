package frc.robot.subsystems.vision.apriltagvision;

import static frc.robot.subsystems.vision.apriltagvision.AprilTagVisionConstants.ROTATION_EULER_MULTIPLIERS;
import static frc.robot.subsystems.vision.apriltagvision.AprilTagVisionConstants.TRANSLATION_EULER_MULTIPLIERS;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

/**
 * This class provides utility methods and record classes for vision-related operations,
 * specifically for pose estimation using April Tags.
 */
public class AprilTagVisionHelpers {
  /**
   * Cool idea by 8033, 5026, and the PoseEstimator docs – dynamically update the vision stddevs
   * based on the distance. In this case, we're scaling the deviations exponentially with distance,
   * so faraway tags are trusted <i>way</i> less than close ones.
   */
  public static Matrix<N3, N1> calculateDeviations(PoseObservation poseObservation) {
    return MatBuilder.fill(
        Nat.N3(),
        Nat.N1(),
        Math.exp(poseObservation.averageTagDistance() / poseObservation.tagCount()));
  }

  public static Matrix<N3, N1> generateDynamicStdDevs(PoseObservation poseObservation) {
    var baseDev =
        poseObservation.ambiguity()
            / poseObservation.tagCount()
            * Math.exp(poseObservation.averageTagDistance() / poseObservation.tagCount());

    var tagCount = MathUtil.clamp(poseObservation.tagCount(), 1, 3);

    var linearStdDev = baseDev * TRANSLATION_EULER_MULTIPLIERS[tagCount - 1].getAsDouble();
    var angularStdDev = baseDev * ROTATION_EULER_MULTIPLIERS[tagCount - 1].getAsDouble();

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
