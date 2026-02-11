
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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.config.BaseConfig;
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
  private final TalonFX indexerMotor = new TalonFX(IndexerConstants.Indexer_ID, Constants.CANIVORE_BUS);
  private final SparkMax angleMotor = new SparkMax(ShooterConstants.angleMotor_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax conveyorMotor = new SparkMax(IndexerConstants.Conveyor_ID, SparkLowLevel.MotorType.kBrushless);
  
  private TalonFXConfiguration bigflytwheelConfig = new TalonFXConfiguration();
  private TalonFXConfiguration smallflywheelConfig = new TalonFXConfiguration();
  private TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

  private final SparkClosedLoopController conveyorPID = conveyorMotor.getClosedLoopController();
  private final SparkClosedLoopController anglePID = angleMotor.getClosedLoopController();

  private final SparkMaxConfig conveyorConfig = new SparkMaxConfig();
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
  bigflytwheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  smallflywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    encoder = angleMotor.getAbsoluteEncoder();
    conveyorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IndexerConstants.conveyor_Run_P, ClosedLoopSlot.kSlot0)
        .i(IndexerConstants.conveyor_Run_I, ClosedLoopSlot.kSlot0)
        .d(IndexerConstants.conveyor_Run_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    conveyorConfig.closedLoop.feedForward
        .kV(IndexerConstants.conveyor_Run_F, ClosedLoopSlot.kSlot0);
    conveyorConfig.closedLoop.maxMotion
        .cruiseVelocity(IndexerConstants.MAX_VELOCITY)
        .maxAcceleration(IndexerConstants.MAX_ACCEL);

    angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.superneo_Back_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.superneo_Back_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.superneo_Back_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop.feedForward
        .kV(ShooterConstants.superneo_Back_F, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.superneo_Out_P, ClosedLoopSlot.kSlot0)
        .i(ShooterConstants.superneo_Out_I, ClosedLoopSlot.kSlot0)
        .d(ShooterConstants.superneo_Out_D, ClosedLoopSlot.kSlot0).maxMotion
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop.feedForward
        .kV(ShooterConstants.superneo_Out_F, ClosedLoopSlot.kSlot0);

    angleConfig.closedLoop.maxMotion
        .cruiseVelocity(ShooterConstants.superneo_MAX_VELOCITY)
        .maxAcceleration(ShooterConstants.superneo_MAX_ACCEL);


    bigflytwheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    smallflywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    indexerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    var bigFlyWheelConfigurator = bigFlyWheel.getConfigurator();
    var smallFlyWheelConfigurator = smallFlyWheel.getConfigurator();
    var indexerConfigurator = indexerMotor.getConfigurator();
   

    bigFlyWheelConfigurator.apply(bigflytwheelConfig);
    smallFlyWheelConfigurator.apply(smallflywheelConfig);
    indexerConfigurator.apply(indexerConfig);


    bigFlyWheel.setNeutralMode(NeutralModeValue.Brake);
    smallFlyWheel.setNeutralMode(NeutralModeValue.Brake);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);


    bigFlyWheelConfigurator
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterB_MAX_ACCEL)
            .withMotionMagicCruiseVelocity(ShooterConstants.ShooterB_MAX_VELOCITY));

    // smallFlyWheelConfigurator
    //     .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ShooterConstants.ShooterS_MAX_ACCEL)
    //         .withMotionMagicCruiseVelocity(ShooterConstants.ShooterS_MAX_VELOCITY));

    indexerConfigurator.apply(new MotionMagicConfigs().withMotionMagicAcceleration(IndexerConstants.MAX_ACCEL)
        .withMotionMagicCruiseVelocity(IndexerConstants.MAX_VELOCITY));



    Slot0Configs BFW_Out_PIDConfig = new Slot0Configs();
    BFW_Out_PIDConfig.kP = ShooterConstants.ShooterB_Out_P;
    BFW_Out_PIDConfig.kI = ShooterConstants.ShooterB_Out_I;
    BFW_Out_PIDConfig.kD = ShooterConstants.ShooterB_Out_D;
    BFW_Out_PIDConfig.kV = ShooterConstants.ShooterB_Out_F;
    bigFlyWheelConfigurator.apply(BFW_Out_PIDConfig);

 


    Slot1Configs Indexerrun_PIDConfig = new Slot1Configs();
    Indexerrun_PIDConfig.kP = IndexerConstants.indexer_Run_P;
    Indexerrun_PIDConfig.kI = IndexerConstants.indexer_Run_I;
    Indexerrun_PIDConfig.kD = IndexerConstants.indexer_Run_D;
    Indexerrun_PIDConfig.kV = IndexerConstants.indexer_Run_F;
    indexerConfigurator.apply(Indexerrun_PIDConfig);



    bigFlyWheel.setPosition(0);
    smallFlyWheel.setPosition(0);
    indexerMotor.setPosition(0);

    smallFlyWheel.setControl(new Follower(bigFlyWheel.getDeviceID(), MotorAlignmentValue.Aligned));

  }



  public void Shooter_Out() {
    bigFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterB_Out).withSlot(0));
    //smallFlyWheel.setControl(new MotionMagicDutyCycle(ShooterConstants.ShooterS_Out).withSlot(0));
  }

  public void stopFlyWheels() {
    bigFlyWheel.stopMotor();
    //smallFlyWheel.stopMotor();
  }

  public void conveyorRun() {
    conveyorPID.setSetpoint(IndexerConstants.conveyor_Run, SparkMax.ControlType.kDutyCycle
                            , ClosedLoopSlot.kSlot0);
  }

  public void stopConveyor() {
    conveyorMotor.stopMotor();
  }

  public void indexerRun(){
    indexerMotor.setControl(new MotionMagicDutyCycle(IndexerConstants.indexer_Run).withSlot(1));
  }

  public void stopIndexer() {
    indexerMotor.stopMotor();
  }

}