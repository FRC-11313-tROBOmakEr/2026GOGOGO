
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX BigFlyWheel = new TalonFX(ShooterConstants.BigFlyWheel_ID, "canbus");
  private final TalonFX SmallFlyWheel = new TalonFX(ShooterConstants.SmallFlyWheel_ID, "canbus");
  private final SparkMax superneo = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax indexerMT = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
  private final SparkMaxConfig indexerconfig = new SparkMaxConfig();
  private final SparkMaxConfig superneoconfig = new SparkMaxConfig();
  private final SparkClosedLoopController posctrl;
  private double angle1;
  public WaitCommand timmer;
  SlewRateLimiter filter = new SlewRateLimiter(0.5);

  private final RelativeEncoder encoder = superneo.getEncoder();

  public Shooter() {
    // 初始化
    SmallFlyWheel.setPosition(0);
    // 回歸原始設定
    BigFlyWheel.setPosition(0);
    // 回歸原始設定

    posctrl = superneo.getClosedLoopController();
    indexerMT.configure(indexerconfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    superneo.configure(superneoconfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // BigFlyWheel.getConfigurator().apply(new TalonFXConfiguration());
    // SmallFlyWheel.getConfigurator().apply(new TalonFXConfiguration());

    var Shooter_Ctrl_Config = BigFlyWheel.getConfigurator();
    Slot0Configs Shooter_Out_PIDConfig = new Slot0Configs();
    Shooter_Out_PIDConfig.kP = ShooterConstants.Shooter_Out_P;
    Shooter_Out_PIDConfig.kI = ShooterConstants.Shooter_Out_I;
    Shooter_Out_PIDConfig.kD = ShooterConstants.Shooter_Out_D;
    Shooter_Out_PIDConfig.kV = ShooterConstants.Shooter_Out_F;
    Shooter_Ctrl_Config.apply(Shooter_Out_PIDConfig);

    Slot0Configs Shooter_Stop_PIDConfig = new Slot0Configs();
    Shooter_Stop_PIDConfig.kP = ShooterConstants.Shooter_Back_P;
    Shooter_Stop_PIDConfig.kI = ShooterConstants.Shooter_Back_I;
    Shooter_Stop_PIDConfig.kD = ShooterConstants.Shooter_Back_D;
    Shooter_Stop_PIDConfig.kV = ShooterConstants.Shooter_Back_F;
    Shooter_Ctrl_Config.apply(Shooter_Stop_PIDConfig);
    // 設定來回的PID值(依齒輪比決定)

    var Shooters_Ctrl_Config = SmallFlyWheel.getConfigurator();
    Slot0Configs Shooters_Out_PIDConfig = new Slot0Configs();
    Shooters_Out_PIDConfig.kP = ShooterConstants.Shooters_Out_P;
    Shooters_Out_PIDConfig.kI = ShooterConstants.Shooters_Out_I;
    Shooters_Out_PIDConfig.kD = ShooterConstants.Shooters_Out_D;
    Shooters_Out_PIDConfig.kV = ShooterConstants.Shooters_Out_F;
    Shooters_Ctrl_Config.apply(Shooter_Out_PIDConfig);

    Slot0Configs Shooters_Stop_PIDConfig = new Slot0Configs();
    Shooters_Stop_PIDConfig.kP = ShooterConstants.Shooters_Back_P;
    Shooters_Stop_PIDConfig.kI = ShooterConstants.Shooters_Back_I;
    Shooters_Stop_PIDConfig.kD = ShooterConstants.Shooters_Back_D;
    Shooters_Stop_PIDConfig.kV = ShooterConstants.Shooters_Back_F;
    Shooters_Ctrl_Config.apply(Shooter_Stop_PIDConfig);
    // 設定來回的PID值(依齒輪比決定)

    // velocityFF 被撇一橫是因為明年就要移除
    superneoconfig.closedLoop.pid(
        ShooterConstants.Shooter_Out_P,
        ShooterConstants.Shooter_Out_I,
        ShooterConstants.Shooter_Out_D);
    superneoconfig.closedLoop.velocityFF(ShooterConstants.Shooter_Out_F);

    superneoconfig.closedLoop.pid(
        ShooterConstants.Shooter_Back_P,
        ShooterConstants.Shooter_Back_I,
        ShooterConstants.Shooter_Back_D);
    superneoconfig.closedLoop.velocityFF(ShooterConstants.Shooter_Back_F);

    indexerconfig.closedLoop.pid(
        IndexerConstants.indexerMT_Out_P,
        IndexerConstants.indexerMT_Out_I,
        IndexerConstants.indexerMT_Out_D);
    indexerconfig.closedLoop.velocityFF(IndexerConstants.indexerMT_Out_F);

    indexerconfig.closedLoop.pid(
        IndexerConstants.indexerMT_Back_P,
        IndexerConstants.indexerMT_Back_I,
        IndexerConstants.indexerMT_Back_D);
    indexerconfig.closedLoop.velocityFF(IndexerConstants.indexerMT_Back_F);
  }

  // 取角度
  public void angle(double angle) {
    posctrl.setSetpoint(angle, SparkMax.ControlType.kPosition);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double limelightMountAngleDegrees = 25.0;
    double limelightLensHeightInches = 20.0;
    double goalHeightInches = 60.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);
    angle = distanceFromLimelightToGoalInches * 0.2;
    angle1 = -angle;

    // 現學物理公式?
    // 高度差?
    // g = 9.8
    // 最大高度要大於砲台? 60 = (初速*sin(angle))^2/2*9.8?
    // (60*2*9.8^1/2)v0 = sin(angle5)
    // 到最高度的距離? distance-2=初速^2*sin(2angle)/2*9.8?
    // 最高設多少?75-15.9626?
    // 是不是還有摩擦力的東東?QAQ

  }

  // 判斷是否在角度
  public Command shoot() {
    angle(angle1);

    if (encoder.getPosition() > posctrl.getSetpoint()) {
      superneo.set(0.2);
      BigFlyWheel.set(filter.calculate(0.5));
      SmallFlyWheel.set(filter.calculate(0.5));
    } else {
      BigFlyWheel.set(filter.calculate(0.5));
      SmallFlyWheel.set(filter.calculate(0.5));
      superneo.set(0);
      indexerMT.set(0.5);
    }
    return null;
  }

  public Command back() {
    posctrl.setSetpoint(angle1, SparkMax.ControlType.kPosition);

    BigFlyWheel.set(0);
    SmallFlyWheel.set(0);
    if (encoder.getPosition() < posctrl.getSetpoint()) {
      superneo.set(-0.2);

    } else {
      superneo.set(0);
      indexerMT.set(0);
    }
    return null;
  }

  public Command shootAuto() {
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> BigFlyWheel.set(filter.calculate(0.5))),
            Commands.runOnce(() -> SmallFlyWheel.set(filter.calculate(0.5)))),
        Commands.waitSeconds(0.8),
        Commands.runOnce(() -> indexerMT.set(0.5)),
        Commands.waitSeconds(3.0),
        Commands.runOnce(() -> {
          BigFlyWheel.set(0);
          SmallFlyWheel.set(0);
          indexerMT.set(0);
        }));
  }
}
