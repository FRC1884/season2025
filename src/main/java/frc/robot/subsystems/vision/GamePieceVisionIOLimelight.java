package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.gamepiecevision.GamePieceVisionConstants.NOTE_CAM_CONSTANTS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Supplier;

public class GamePieceVisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public GamePieceVisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    table
        .getDoubleArrayTopic("camerapose_robotspace_set")
        .publish()
        .accept(
            new double[] {
              NOTE_CAM_CONSTANTS.robotToCamera().getX(),
              NOTE_CAM_CONSTANTS.robotToCamera().getY(),
              NOTE_CAM_CONSTANTS.robotToCamera().getZ(),
              NOTE_CAM_CONSTANTS.robotToCamera().getRotation().getX(),
              NOTE_CAM_CONSTANTS.robotToCamera().getRotation().getY(),
              NOTE_CAM_CONSTANTS.robotToCamera().getRotation().getZ()
            });
  }

  @Override
  public void updateInputs(VisionIO.VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;
    // Update target observation
    inputs.latestTargetObservation =
        new VisionIO.TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
