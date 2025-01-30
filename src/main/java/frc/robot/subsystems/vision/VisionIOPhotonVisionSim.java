package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.GlobalConstants.FieldMap;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param cameraConstants The constants class associated with the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(CameraConstants cameraConstants, Supplier<Pose2d> poseSupplier) {
    super(cameraConstants);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldMap.APRIL_TAG_FIELD_LAYOUT);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibError(0.5, 0.5);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIO.VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
