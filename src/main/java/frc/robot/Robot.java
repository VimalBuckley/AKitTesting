// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.mechanisms.flywheels.FlywheelMechanism;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.TalonFXMotor;
import frc.robot.utilities.FeedbackController.PIDFeedback;
import frc.robot.utilities.FeedbackController.ProfiledFeedback;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static ArrayList<Double> supplyCurrents;
  public static ArrayList<MotorIO> ios;
  private FlywheelMechanism flywheel;

  public Robot() {
    setupAdvantageKit();
    new ProfiledFeedback(5, new Constraints(4, 5));
    supplyCurrents = new ArrayList<>();
    ios = new ArrayList<>();
    flywheel =
        new FlywheelMechanism(
            new TalonFXMotor(
                "Test Flywheel/Spin Motor", 1, new TalonFXConfiguration(), DCMotor.getKrakenX60(1)),
            1,
            1,
            0.01,
            new PIDFeedback(0));
  }

  @Override
  public void teleopInit() {
    flywheel.setTargetSpeed(300);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    for (MotorIO io : ios) {
      io.updateInputs();
    }
    CommandScheduler.getInstance().run();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    double[] currents = new double[supplyCurrents.size()];
    for (int i = 0; i < currents.length; i++) {
      currents[i] = supplyCurrents.get(i);
    }
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currents));
  }

  public void setupAdvantageKit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }
}
