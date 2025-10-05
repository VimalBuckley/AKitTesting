package frc.robot.hardware.tagCameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utilities.Loggable;

/** A class used to interface with camers that detect AprilTags */
public abstract class TagCameraIO implements Loggable {
  /**
   * A class that represents a position estimate of the robot obtained by a camera
   *
   * @param pose The estimated position of the robot
   * @param latencySeconds The approximate time since this estimate was taken
   * @param exists Whether this actually contains a valid position estimate
   */
  public static record PoseEstimate(Pose2d pose, double latencySeconds, boolean exists) {}

  /**
   * A class that represents an april tag reading obtained by a camera
   *
   * @param robotToTagTransform The estimated position of the seen tag relative to the robot
   * @param tagID The ID of the seen tag
   * @param exists Whether this actually contains a valid tag reading
   */
  public static record TagReading(Transform3d robotToTagTransform, int tagID, boolean exists) {}

  /**
   * @return The latest pose estimate from the camera
   */
  public abstract PoseEstimate getPoseEstimate();

  /**
   * @return The latest tag reading from the camera
   */
  public abstract TagReading getTagReading();
}
