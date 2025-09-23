package frc.robot.utilities;

public interface Loggable {
  public void log(String name);

  public default void log(String path, String name) {
    log(path + "/" + name);
  }
}
