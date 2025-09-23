package frc.robot.hardware.tagCameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utilities.Loggable;

public abstract class TagCameraIO implements Loggable {
  public static record PoseEstimate(Pose2d pose, double latencySeconds, boolean exists) {}

  public static record TagReading(Transform3d robotToTagTransform, int tagID, boolean exists) {}

  public abstract PoseEstimate getPoseEstimate();

  public abstract TagReading getTagReading();
}
