package frc.robot.hardware;

/**
 * Represents a generic input to the robot program. Because we want to utilize advantage kit's log
 * replay funcationality, all inputs must be seperated out so that they can be fed into the robot
 * program to deterministically replay our code
 *
 * @see <a href =
 *     "https://docs.advantagekit.org/getting-started/common-issues/non-deterministic-data-sources">Advantage
 *     Kit Docs</a>
 */
public interface IOLayer {
  /**
   * Updates the hardware values that are read by the rest of the code. This should only be called
   * once per loop!
   */
  public void updateInputs();
}
