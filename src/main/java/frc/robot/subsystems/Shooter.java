
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import frc.robot.LimelightHelpers;
import frc.robot.Target;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX bigFlyWheel = new TalonFX(ShooterConstants.BigFlyWheel_ID, Constants.CANIVORE_BUS);
  private final TalonFX smallFlyWheel = new TalonFX(ShooterConstants.SmallFlyWheel_ID, Constants.CANIVORE_BUS);
  private final TalonFX indexerMotor = new TalonFX(IndexerConstants.IndexerMT2_ID, Constants.CANIVORE_BUS);
  private final SparkMax angleMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax conveyorMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);

  private TalonFXConfiguration baseConfig = new TalonFXConfiguration();
  private final SparkClosedLoopController conveyorPID = conveyorMotor.getClosedLoopController();

  private final SparkMaxConfig indexerConfig = new SparkMaxConfig();
  private final SparkMaxConfig angleConfig = new SparkMaxConfig();

  private final Target target = new Target();

  private SparkAbsoluteEncoder encoder;

  public WaitCommand timmer;
  public double smoothSpeed;
  public double targetSpeed = 0.8;
  private double angle;
  public Pose2d TargetPose = new Pose2d();
  public static HashMap<Integer, List<Pose2d>> hubMap = new HashMap<>();

  public Shooter() {
    encoder = angleMotor.getAbsoluteEncoder();
    indexerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.indexer_Back_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.indexer_Back_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.indexer_Back_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    indexerConfig.closedLoop.feedForward
        .kV(ShooterConstants.indexer_Back_F, ClosedLoopSlot.kSlot0);

    indexerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.indexer_Out_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.indexer_Out_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.indexer_Out_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    indexerConfig.closedLoop.feedForward
        .kV(ShooterConstants.indexer_Out_F, ClosedLoopSlot.kSlot0);

    indexerConfig.closedLoop.maxMotion
        .cruiseVelocity(ShooterConstants.indexer_MAX_VELOCITY)
        .maxAcceleration(ShooterConstants.indexer_MAX_ACCEL);

    angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.superneo_Back_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.superneo_Back_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.superneo_Back_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop.feedForward
        .kV(ShooterConstants.superneo_Back_F, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(ShooterConstants.superneo_Out_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.superneo_Out_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.superneo_Out_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop.feedForward
        .kV(ShooterConstants.superneo_Out_F, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop.maxMotion
        .cruiseVelocity(ShooterConstants.superneo_MAX_VELOCITY)
        .maxAcceleration(ShooterConstants.superneo_MAX_ACCEL);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    conveyorMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // revertConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    baseConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    baseConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    var bigFlyWheelConfigurator = bigFlyWheel.getConfigurator();
    var smallFlyWheelConfigurator = smallFlyWheel.getConfigurator();
    var indexerConfigurator = indexerMotor.getConfigurator();

    bigFlyWheelConfigurator.apply(baseConfig);
    smallFlyWheelConfigurator.apply(baseConfig);
    indexerConfigurator.apply(baseConfig);

    bigFlyWheel.setNeutralMode(NeutralModeValue.Brake);
    smallFlyWheel.setNeutralMode(NeutralModeValue.Brake);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);

    bigFlyWheelConfigurator
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterB_MAX_ACCEL)
            .withMotionMagicCruiseVelocity(ShooterConstants.ShooterB_MAX_VELOCITY));

    smallFlyWheelConfigurator
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterS_MAX_ACCEL)
            .withMotionMagicCruiseVelocity(ShooterConstants.ShooterS_MAX_VELOCITY));

    indexerConfigurator.apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterS_MAX_ACCEL)
        .withMotionMagicCruiseVelocity(ShooterConstants.indexer_MAX_VELOCITY));

    Slot0Configs BFW_Out_PIDConfig = new Slot0Configs();
    BFW_Out_PIDConfig.kP = ShooterConstants.ShooterB_Out_P;
    BFW_Out_PIDConfig.kI = ShooterConstants.ShooterB_Out_I;
    BFW_Out_PIDConfig.kD = ShooterConstants.ShooterB_Out_D;
    BFW_Out_PIDConfig.kV = ShooterConstants.ShooterB_Out_F;
    bigFlyWheelConfigurator.apply(BFW_Out_PIDConfig);

    Slot1Configs BFW_Back_PIDConfig = new Slot1Configs();
    BFW_Back_PIDConfig.kP = ShooterConstants.ShooterB_Back_P;
    BFW_Back_PIDConfig.kI = ShooterConstants.ShooterB_Back_I;
    BFW_Back_PIDConfig.kD = ShooterConstants.ShooterB_Back_D;
    BFW_Back_PIDConfig.kV = ShooterConstants.ShooterB_Back_F;
    bigFlyWheelConfigurator.apply(BFW_Back_PIDConfig);

    Slot0Configs SFW_Out_PIDConfig = new Slot0Configs();
    SFW_Out_PIDConfig.kP = ShooterConstants.ShooterS_Out_P;
    SFW_Out_PIDConfig.kI = ShooterConstants.ShooterS_Out_I;
    SFW_Out_PIDConfig.kD = ShooterConstants.ShooterS_Out_D;
    SFW_Out_PIDConfig.kV = ShooterConstants.ShooterS_Out_F;
    smallFlyWheelConfigurator.apply(SFW_Out_PIDConfig);

    Slot1Configs SFW_Back_PIDConfig = new Slot1Configs();
    SFW_Back_PIDConfig.kP = ShooterConstants.ShooterS_Back_P;
    SFW_Back_PIDConfig.kI = ShooterConstants.ShooterS_Back_I;
    SFW_Back_PIDConfig.kD = ShooterConstants.ShooterS_Back_D;
    SFW_Back_PIDConfig.kV = ShooterConstants.ShooterS_Back_F;
    smallFlyWheelConfigurator.apply(SFW_Back_PIDConfig);

    Slot1Configs Indexerrun_PIDConfig = new Slot1Configs();
    Indexerrun_PIDConfig.kP = ShooterConstants.IndexerMT2run_P;
    Indexerrun_PIDConfig.kI = ShooterConstants.IndexerMT2run_I;
    Indexerrun_PIDConfig.kD = ShooterConstants.IndexerMT2run_D;
    Indexerrun_PIDConfig.kV = ShooterConstants.IndexerMT2run_F;
    indexerConfigurator.apply(Indexerrun_PIDConfig);

    bigFlyWheel.setPosition(0);
    smallFlyWheel.setPosition(0);
    indexerMotor.setPosition(0);
  }

  public double getPositionbig() {
    return bigFlyWheel.getPosition().getValueAsDouble();
  }

  public double getPositionsmall() {
    return smallFlyWheel.getPosition().getValueAsDouble();
  }

  // Intake Position

  public void Shooter_Out() {
    bigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterB_Out).withSlot(0));
    smallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterS_Out).withSlot(0));
  }

  public void stopFlyWheels() {
    bigFlyWheel.stopMotor();
    smallFlyWheel.stopMotor();
  }

  // TODO: 改成 Command？
  // 取角度
  public void angle_out() {
    // jocker->學長、婉溱、宥云、盈萱
    // if (LimelightHelpers.getTV(VisionConstants.LLName)) {}
    angle = target.getDistanceToTarget(LimelightHelpers.getBotPose2d(VisionConstants.LLName)) * 0.3 + 0.145;

    if (encoder.getPosition() - angle > 0.5) {
      angleMotor.set(0);
      angle = -angle;
    } else {
      angleMotor.set(0.2);
    }
  }

  // TODO: 改成 Command？
  public void angle_in() {
    if (encoder.getPosition() - angle > 0.5) {
      angleMotor.set(0);
    } else {
      angleMotor.set(-0.2);
    }
  }

  public void conveyorRun() {
    // TODO: 寫出 conveyor 功能
  }

  public void stopConveyor() {
    conveyorMotor.stopMotor();
  }

}