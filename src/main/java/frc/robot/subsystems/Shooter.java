
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import com.revrobotics.RelativeEncoder;

import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Target;

public class Shooter extends SubsystemBase {
  private final TalonFX BigFlyWheel = new TalonFX(ShooterConstants.BigFlyWheel_ID, Constants.CANIVORE_BUS);
  private final TalonFX SmallFlyWheel = new TalonFX(ShooterConstants.SmallFlyWheel_ID, Constants.CANIVORE_BUS);
  TalonFXConfiguration configuration = new TalonFXConfiguration();

  private final SparkMax superneo = new SparkMax(2, SparkLowLevel.MotorType.kBrushless); // 旋轉角度
  private final SparkMax indexerMT = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);

  private final SparkClosedLoopController superneoPID = superneo.getClosedLoopController();
  private final SparkClosedLoopController indexerMTPID = indexerMT.getClosedLoopController();

  private final SparkMaxConfig indexerconfig = new SparkMaxConfig();
  private final SparkMaxConfig superneoconfig = new SparkMaxConfig();

  private final Target target = new Target();

  private RelativeEncoder encoder;

  public WaitCommand timmer;
  private Pose3d relativePoseToTag;
  private double distanceToTag;
  public double smoothSpeed;
  public double targetSpeed = 0.8;
  private double angle1;

  public Shooter() {
    indexerconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.indexer_Back_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.indexer_Back_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.indexer_Back_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    indexerconfig.closedLoop.feedForward
        .kV(ShooterConstants.indexer_Back_F, ClosedLoopSlot.kSlot0);

    indexerconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.indexer_Out_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.indexer_Out_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.indexer_Out_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    indexerconfig.closedLoop.feedForward
        .kV(ShooterConstants.indexer_Out_F, ClosedLoopSlot.kSlot0);

    indexerconfig.closedLoop.maxMotion
        .cruiseVelocity(ShooterConstants.indexer_MAX_VELOCITY)
        .maxAcceleration(ShooterConstants.indexer_MAX_ACCEL);

    superneoconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.superneo_Back_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.superneo_Back_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.superneo_Back_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    superneoconfig.closedLoop.feedForward
        .kV(ShooterConstants.superneo_Back_F, ClosedLoopSlot.kSlot0);

    superneoconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.superneo_Out_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.superneo_Out_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.superneo_Out_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    superneoconfig.closedLoop.feedForward
        .kV(ShooterConstants.superneo_Out_F, ClosedLoopSlot.kSlot0);

    superneoconfig.closedLoop.maxMotion
        .cruiseVelocity(ShooterConstants.superneo_MAX_VELOCITY)
        .maxAcceleration(ShooterConstants.superneo_MAX_ACCEL);

    superneo.configure(superneoconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    indexerMT.configure(indexerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var BigFlyWheelConfig = BigFlyWheel.getConfigurator();
    var SmallFlyWheelConfig = SmallFlyWheel.getConfigurator();

    TalonFXConfiguration bigFlyWheelConfigs = new TalonFXConfiguration();
    TalonFXConfiguration smallFlyWheelConfigs = new TalonFXConfiguration();

    BigFlyWheel.getConfigurator().apply(bigFlyWheelConfigs);
    SmallFlyWheel.getConfigurator().apply(smallFlyWheelConfigs);

    BigFlyWheel.setNeutralMode(NeutralModeValue.Brake);
    SmallFlyWheel.setNeutralMode(NeutralModeValue.Brake);

    BigFlyWheelConfig.apply(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    BigFlyWheelConfig.apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterB_MAX_ACCEL)
        .withMotionMagicCruiseVelocity(ShooterConstants.ShooterB_MAX_VELOCITY));

    SmallFlyWheelConfig.apply(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    SmallFlyWheelConfig.apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterS_MAX_ACCEL)
        .withMotionMagicCruiseVelocity(ShooterConstants.ShooterS_MAX_VELOCITY));

    Slot0Configs BFW_Out_PIDConfig = new Slot0Configs();
    BFW_Out_PIDConfig.kP = ShooterConstants.ShooterB_Out_P;
    BFW_Out_PIDConfig.kI = ShooterConstants.ShooterB_Out_I;
    BFW_Out_PIDConfig.kD = ShooterConstants.ShooterB_Out_D;
    BFW_Out_PIDConfig.kV = ShooterConstants.ShooterB_Out_F;
    BigFlyWheel.getConfigurator().apply(BFW_Out_PIDConfig);

    Slot1Configs BFW_Back_PIDConfig = new Slot1Configs();
    BFW_Back_PIDConfig.kP = ShooterConstants.ShooterB_Back_P;
    BFW_Back_PIDConfig.kI = ShooterConstants.ShooterB_Back_I;
    BFW_Back_PIDConfig.kD = ShooterConstants.ShooterB_Back_D;
    BFW_Back_PIDConfig.kV = ShooterConstants.ShooterB_Back_F;
    BigFlyWheel.getConfigurator().apply(BFW_Back_PIDConfig);

    BigFlyWheelConfig.setPosition(0);

    Slot0Configs SFW_Out_PIDConfig = new Slot0Configs();
    SFW_Out_PIDConfig.kP = ShooterConstants.ShooterS_Out_P;
    SFW_Out_PIDConfig.kI = ShooterConstants.ShooterS_Out_I;
    SFW_Out_PIDConfig.kD = ShooterConstants.ShooterS_Out_D;
    SFW_Out_PIDConfig.kV = ShooterConstants.ShooterS_Out_F;
    SmallFlyWheel.getConfigurator().apply(SFW_Out_PIDConfig);

    Slot1Configs SFW_Back_PIDConfig = new Slot1Configs();
    SFW_Back_PIDConfig.kP = ShooterConstants.ShooterS_Back_P;
    SFW_Back_PIDConfig.kI = ShooterConstants.ShooterS_Back_I;
    SFW_Back_PIDConfig.kD = ShooterConstants.ShooterS_Back_D;
    SFW_Back_PIDConfig.kV = ShooterConstants.ShooterS_Back_F;
    SmallFlyWheel.getConfigurator().apply(SFW_Back_PIDConfig);
    SmallFlyWheelConfig.setPosition(0);
  }

  public double getPositionbig() {
    return BigFlyWheel.getPosition().getValueAsDouble();
  }

  public double getPositionsmall() {
    return SmallFlyWheel.getPosition().getValueAsDouble();
  }

  // Intake Position
  public void Shooter_Zero(double rpm) {
    BigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterB_Zero));
    SmallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterS_Zero));
    superneoPID.setSetpoint(rpm, SparkMax.ControlType.kMAXMotionVelocityControl);
    indexerMTPID.setSetpoint(rpm, SparkMax.ControlType.kMAXMotionVelocityControl);
  }

  public void Shooter_Out() {
    BigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterB_Out).withSlot(0));
    SmallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterS_Out).withSlot(0));
    superneoPID.setSetpoint(ShooterConstants.superneo_Out, SparkMax.ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0);
    indexerMTPID.setSetpoint(ShooterConstants.indexer_Out, SparkMax.ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0);

  }

  public void Shooter_Back() {
    BigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterB_Back).withSlot(1));
    SmallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterS_Back).withSlot(1));
    superneoPID.setSetpoint(ShooterConstants.superneo_Back, SparkMax.ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0);
    indexerMTPID.setSetpoint(ShooterConstants.indexer_Back, SparkMax.ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0);
  }

  public void Encoder() {
    encoder = superneo.getEncoder();

  }

  // 取角度
 public void angle_out(double angle) {
    // jocker->學長、婉溱、宥云、盈萱
    // if (LimelightHelpers.getTV(VisionConstants.LLName)) {}
    angle = target.getDistanceToTarget(LimelightHelpers.getBotPose2d("LL"))*0.3+0.145;
    angle1 = -angle;
    if (encoder.getPosition() - angle > 0.5) {
      superneo.set(0);
    }else{
      superneo.set(0.2);
    };
  }
  
  public void angle_in(double angle) {

    if (encoder.getPosition() - angle1 > 0.5) {
      superneo.set(0);
    }else{
      superneo.set(-0.2);
    };
  }

  public void InxererWorking() {
    indexerMT.set(0.6);
  }

  public void IndexerStop() {
    indexerMT.set(0);
  }
}