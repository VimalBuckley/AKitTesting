package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.WiringConstants.SwerveWiring;
import frc.robot.hardware.encoders.AnalogEncoderIO;
import frc.robot.hardware.mechanisms.swerve.SwerveModule;
import frc.robot.hardware.motors.MotorIO;
import frc.robot.hardware.motors.MotorIOConfig;
import frc.robot.utilities.FeedbackController.PIDFeedback;

public class SwerveConstants {
  private static final MotorIOConfig DRIVE_CONFIG = new MotorIOConfig()
    .withStatorLimit(60)
    .withSupplyLimit(60)
    .withNeutralMode(NeutralModeValue.Brake)
    .withOutputRange(-12, 12);

  private static final MotorIOConfig ANGLE_CONFIG = new MotorIOConfig()
    .withStatorLimit(20)
    .withSupplyLimit(20)
    .withInversion(InvertedValue.CounterClockwise_Positive)
    .withNeutralMode(NeutralModeValue.Coast)
    .withOutputRange(-12, 12);

  public static final SwerveModule FRONT_LEFT_MODULE =
      SwerveModule.fromMeasuredValues(
          new Translation2d(0.368, 0.266),
          0.0467733641327,
          MotorIO.makeTalonFX(
              SwerveWiring.FRONT_LEFT_DRIVE_ID,
              new CANBus(),
              DCMotor.getKrakenX60(1),
              DRIVE_CONFIG.withInversion(InvertedValue.Clockwise_Positive)),
          new PIDFeedback(1.96850393701),
          5.143,
          1,
          0.19635,
          0.10308336,
          0.009937496,
          MotorIO.makeSparkMax(
              SwerveWiring.FRONT_LEFT_ANGLE_ID, 
              DCMotor.getNeo550(1),
              ANGLE_CONFIG),
          new PIDFeedback(5.72957795131),
          25,
          1,
          0.25348,
          0.491242554389,
          0.0619252784977,
          new AnalogEncoderIO(SwerveWiring.FRONT_LEFT_ENCODER_ID, 4.033805, encoder -> {}));

  public static final SwerveModule FRONT_RIGHT_MODULE =
      SwerveModule.fromMeasuredValues(
          new Translation2d(0.368, -0.266),
          0.0467733641327,
          MotorIO.makeTalonFX(
              SwerveWiring.FRONT_RIGHT_DRIVE_ID,
              new CANBus(),
              DCMotor.getKrakenX60(1),
              DRIVE_CONFIG.withInversion(InvertedValue.CounterClockwise_Positive)),
          new PIDFeedback(1.96850393701),
          5.143,
          1,
          0.20427,
          0.10233152,
          0.012937236,
          MotorIO.makeSparkMax(
              SwerveWiring.FRONT_RIGHT_ANGLE_ID, 
              DCMotor.getNeo550(1),
              ANGLE_CONFIG),
          new PIDFeedback(5.72957795131),
          25,
          1,
          0.27701,
          0.515003114153,
          0.0627675264566,
          new AnalogEncoderIO(SwerveWiring.FRONT_RIGHT_ENCODER_ID, 4.1971678, encoder -> {}));

  public static final SwerveModule BACK_LEFT_MODULE =
      SwerveModule.fromMeasuredValues(
          new Translation2d(-0.368, 0.266),
          0.0467733641327,
          MotorIO.makeTalonFX(
              SwerveWiring.BACK_LEFT_DRIVE_ID, 
              new CANBus(),
              DCMotor.getKrakenX60(1),
              DRIVE_CONFIG.withInversion(InvertedValue.Clockwise_Positive)),
          new PIDFeedback(1.96850393701),
          5.143,
          1,
          0.2049,
          0.10245852,
          0.01343152,
          MotorIO.makeSparkMax(
              SwerveWiring.BACK_LEFT_ANGLE_ID, 
              DCMotor.getNeo550(1),
              ANGLE_CONFIG),
          new PIDFeedback(5.72957795131),
          25,
          1,
          0.25886,
          0.520658207591,
          0.0725479160195,
          new AnalogEncoderIO(SwerveWiring.BACK_LEFT_ENCODER_ID, 0.13823008, encoder -> {}));

  public static final SwerveModule BACK_RIGHT_MODULE =
      SwerveModule.fromMeasuredValues(
          new Translation2d(-0.368, -0.266),
          0.0467733641327,
          MotorIO.makeTalonFX(
              SwerveWiring.BACK_RIGHT_DRIVE_ID,
              new CANBus(),
              DCMotor.getKrakenX60(1),
              DRIVE_CONFIG.withInversion(InvertedValue.CounterClockwise_Positive)),
          new PIDFeedback(1.96850393701),
          5.143,
          1,
          0.20206,
          0.10634472,
          0.009241536,
          MotorIO.makeSparkMax(
              SwerveWiring.BACK_RIGHT_ANGLE_ID, 
              DCMotor.getNeo550(1),
              ANGLE_CONFIG),
          new PIDFeedback(5.72957795131),
          25,
          1,
          0.25348,
          0.528765560392,
          0.0818699393462,
          new AnalogEncoderIO(SwerveWiring.BACK_RIGHT_ENCODER_ID, 5.5229199, encoder -> {}));
}
