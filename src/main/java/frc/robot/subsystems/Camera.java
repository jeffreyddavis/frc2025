package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
  public Pose3d getPose3d(Pose3d cameraPose) {
    PhotonCamera camera = new PhotonCamera("pho_noodles");

    var result = camera.getLatestResult();
    camera.close();
    if (!result.hasTargets()) return null;
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();

    return cameraPose.plus(bestCameraToTarget);
  }
}
