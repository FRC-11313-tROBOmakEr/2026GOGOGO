
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import frc.robot.LimelightHelpers;



public class Shooter extends SubsystemBase {
  private final TalonFX BigFlyWheel = new TalonFX(0);
  private final TalonFX SmallFlyWheel = new TalonFX(1);
  private final SparkMax superneo = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax indexerMT = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkClosedLoopController posctrl;
  private double angle1;

  private final RelativeEncoder encoder = superneo.getEncoder();

  public Shooter() {
    // 初始化哈哈哈
    posctrl = superneo.getClosedLoopController();
    indexerMT.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    superneo.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  // 取角度哈哈哈
  public void angle(double angle) {
    posctrl.setSetpoint(angle, ControlType.kPosition);
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
    angle = distanceFromLimelightToGoalInches;
    angle1 = -angle;
    // 現學物理公式?
    // 高度差哈哈哈?
    // g = 9.8
    // 最大高度要大於砲台? 60 = (初速*sin(angle))^2/2*9.8?
    // (60*2*9.8^1/2)v0 = sin(angle5)
    // 到最高度的距離? distance-2=初速^2*sin(2angle)/2*9.8?
    // 最高設多少?75-15.9626?
    // 是不是還有摩擦力的東東?QAQ

  }

  // 判斷是否在角度哈哈哈
  public Command shoot() {
    angle(angle1);

    if (!posctrl.isAtSetpoint()) {
      superneo.set(0.2);
      for (int i = 0; i <= 0.5; i += 0.05) {
        BigFlyWheel.set(i);
        SmallFlyWheel.set(i);
        Timer.delay(0.01);

      }
    } 
    else {
      BigFlyWheel.set(0.5);
      SmallFlyWheel.set(0.5);
      superneo.set(0);
      indexerMT.set(0.5);
    }

    return null;
  }

  public Command back() {
    posctrl.setSetpoint(angle1, ControlType.kPosition);

    BigFlyWheel.set(0);
    SmallFlyWheel.set(0);
    if (!posctrl.isAtSetpoint()) {
      superneo.set(-0.2);

    } else {
      superneo.set(0);
      indexerMT.set(0);
    }
    return null;
  }
}
