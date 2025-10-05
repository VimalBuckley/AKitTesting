package frc.robot.utilities;

/** A interface that marks an object as being loggable */
public interface Loggable {
  /**
   * Logs information about this object, and calls the log method on all {@link Loggable} member
   * variables
   *
   * @param name The name for this object to be logged under
   */
  public void log(String name);

  /**
   * Logs information about this object, and calls the log method on all {@link Loggable} member
   * variables
   *
   * @param path The folder path to log this object under
   * @param name The name this object will be logged under
   */
  public default void log(String path, String name) {
    log(path + "/" + name);
  }
}
