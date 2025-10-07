package frc.robot.hardware.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.utilities.Loggable;
import java.util.Optional;

public abstract class Mechanism<CurrentState, TargetState> extends SubsystemBase
    implements Loggable {
  private Optional<TargetState> target;
  private boolean enabled;
  private int simsPerLoop;

  public Mechanism(int simsPerLoop) {
    enabled = false;
  }

  public abstract CurrentState getState();

  protected abstract void updateSim(double dt);

  protected abstract void moveToTarget();

  public void setTarget(TargetState target) {
    setTarget(target, true);
  }

  public void setTarget(TargetState target, boolean enable) {
    this.target = Optional.of(target);
    this.enabled = enable;
  }

  public void enableMechanism() {
    enabled = true;
  }

  public void disableMechanism() {
    enabled = false;
  }

  public Optional<TargetState> getTarget() {
    return target;
  }

  @Override
  public final void periodic() {
    if (!enabled || target.isEmpty()) {
      return;
    }
    moveToTarget();
  }

  @Override
  public void simulationPeriodic() {
    if (Constants.currentMode != Mode.SIM) {
      return;
    }
    for (int i = 0; i < simsPerLoop; i++) {
      updateSim(0.02 / i);
    }
  }
}
