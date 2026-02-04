
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX BigFlyWheel = new TalonFX(ShooterConstants.BigFlyWheel_ID, "canbus");
  private final TalonFX SmallFlyWheel = new TalonFX(ShooterConstants.SmallFlyWheel_ID, "canbus");
  TalonFXConfiguration configuration = new TalonFXConfiguration();

  private final SparkMax superneo = new SparkMax(2, SparkLowLevel.MotorType.kBrushless); // 旋轉角度
  private final SparkMax indexerMT = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);

  private final SparkClosedLoopController superneoPID = superneo.getClosedLoopController();
  private final SparkClosedLoopController conveyorPID = indexerMT.getClosedLoopController();

  private final SparkMaxConfig indexerconfig = new SparkMaxConfig();
  private final SparkMaxConfig superneoconfig = new SparkMaxConfig();

  private RelativeEncoder encoder;

  private final VoltageOut m_request = new VoltageOut(0);

  private double angle1;
  public WaitCommand timmer;
  private Pose3d relativePoseToTag;
  private double distanceToTag;
  public double smoothSpeed;
  public double targetSpeed = 0.8;
  private SparkClosedLoopController posctrl;

  public Shooter() {

    indexerconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001)
        .i(0.0001)
        .d(0.001)
        .velocityFF(0.00017).maxMotion
        .maxVelocity(2000) // RPM
        .maxAcceleration(1500) // RPM/s
        .allowedClosedLoopError(0.05);

    superneoconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001)
        .i(0.0001)
        .d(0.001)
        .velocityFF(0.00017).maxMotion
        .maxVelocity(2000) // RPM
        .maxAcceleration(1500) // RPM/s
        .allowedClosedLoopError(0.05);

    superneo.configure(superneoconfig, SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
    indexerMT.configure(indexerconfig, SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var BigFlyWheelConfig = BigFlyWheel.getConfigurator();
    var SmallFlyWheelConfig = SmallFlyWheel.getConfigurator();

    TalonFXConfiguration bigFlyWheelConfigs = new TalonFXConfiguration();
    TalonFXConfiguration smallFlyWheelConfigs = new TalonFXConfiguration();

    BigFlyWheel.getConfigurator().apply(bigFlyWheelConfigs);
    SmallFlyWheel.getConfigurator().apply(smallFlyWheelConfigs);

    BigFlyWheel.setNeutralMode(NeutralModeValue.Brake);
    SmallFlyWheel.setNeutralMode(NeutralModeValue.Brake);

    BigFlyWheelConfig.apply(new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    BigFlyWheelConfig.apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(ShooterConstants.MAX_ACCEL)
        .withMotionMagicCruiseVelocity(ShooterConstants.MAX_VELOCITY));

    SmallFlyWheelConfig.apply(new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    SmallFlyWheelConfig.apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(ShooterConstants.MAX_ACCEL)
        .withMotionMagicCruiseVelocity(ShooterConstants.MAX_VELOCITY));

    Slot0Configs BFW_Out_PIDConfig = new Slot0Configs();
    BFW_Out_PIDConfig.kP = ShooterConstants.Shooterb_Out_P;
    BFW_Out_PIDConfig.kI = ShooterConstants.Shooterb_Out_I;
    BFW_Out_PIDConfig.kD = ShooterConstants.Shooterb_Out_D;
    BFW_Out_PIDConfig.kV = ShooterConstants.Shooterb_Out_F;
    BigFlyWheel.getConfigurator().apply(BFW_Out_PIDConfig);

    Slot0Configs BFW_Back_PIDConfig = new Slot0Configs();
    BFW_Back_PIDConfig.kP = ShooterConstants.Shooterb_Back_P;
    BFW_Back_PIDConfig.kI = ShooterConstants.Shooterb_Back_I;
    BFW_Back_PIDConfig.kD = ShooterConstants.Shooterb_Back_D;
    BFW_Back_PIDConfig.kV = ShooterConstants.Shooterb_Back_F;
    BigFlyWheel.getConfigurator().apply(BFW_Back_PIDConfig);

    BigFlyWheelConfig.setPosition(0);

    Slot0Configs SFW_Out_PIDConfig = new Slot0Configs();
    SFW_Out_PIDConfig.kP = ShooterConstants.Shooters_Out_P;
    SFW_Out_PIDConfig.kI = ShooterConstants.Shooters_Out_I;
    SFW_Out_PIDConfig.kD = ShooterConstants.Shooters_Out_D;
    SFW_Out_PIDConfig.kV = ShooterConstants.Shooters_Out_F;
    SmallFlyWheel.getConfigurator().apply(BFW_Out_PIDConfig);

    Slot0Configs SFW_Back_PIDConfig = new Slot0Configs();
    SFW_Back_PIDConfig.kP = ShooterConstants.Shooters_Back_P;
    SFW_Back_PIDConfig.kI = ShooterConstants.Shooters_Back_I;
    SFW_Back_PIDConfig.kD = ShooterConstants.Shooters_Back_D;
    SFW_Back_PIDConfig.kV = ShooterConstants.Shooters_Back_F;
    SmallFlyWheel.getConfigurator().apply(BFW_Back_PIDConfig);
    SmallFlyWheelConfig.setPosition(0);
  }

  public double getPositionbig() {
    return BigFlyWheel.getPosition().getValueAsDouble();
  }

  public double getPositionsmall() {
    return SmallFlyWheel.getPosition().getValueAsDouble();
  }

  // Intake Position
  public void Shooter_Zero() {
    BigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.Shooter_Zero));
    SmallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.Shooter_Zero));
  }

  public void Shooter_Out() {
    BigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.Shooter_StartUp));
    SmallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.Shooter_StartUp));
  }

  public void Shooter_Back() {
    BigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.BigFlyWheel_ID).withSlot(1));
    SmallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.SmallFlyWheel_ID).withSlot(1));
  }

  public void Encoder() {
    encoder = superneo.getEncoder();

  }

  // 取角度
  public void angle(double angle) {
    posctrl.setSetpoint(angle, SparkMax.ControlType.kPosition);
    // Hint: Use LimelightHelpers.getBotPose3d_TargetSpace() to get x offset & y
    // offset, then calculate to get distance
    // jocker->學長
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Translation2d.html#getDistance(edu.wpi.first.math.geometry.Translation2d)

    relativePoseToTag = LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.LLName);
    distanceToTag = Math.sqrt(Math.pow(relativePoseToTag.getX(), 2) + Math.pow(relativePoseToTag.getY(), 2));
    // 學長賽高!!!
    angle = distanceToTag * 0.0475 + 10.36;
    angle1 = -angle;
  }
}