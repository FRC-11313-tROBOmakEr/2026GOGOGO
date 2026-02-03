
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.VoltageOut;

public class Climber extends SubsystemBase {
  private TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotor_ID, "canbus");
  private TalonFX tubeMotor1 = new TalonFX(ClimberConstants.tubeMotor1_ID, "canbus");
  private TalonFX tubeMotor2 = new TalonFX(ClimberConstants.tubeMotor2_ID, "canbus");
  private Follower follower;

  public void Init() {

    // var talonFXConfigs = new TalonFXConfiguration();
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor1.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor2.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor2.setControl(follower.withLeaderID(1));
    // (可能要反轉)config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // climberMotor.setPosition(0);
    // 回歸原始設定
    // climberMotor.setPosition(0);
    // 回歸原始設定

    var Climber_Ctrl_Config = climberMotor.getConfigurator();
    Slot0Configs Climber_Out_PIDConfig = new Slot0Configs();
    Climber_Out_PIDConfig.kP = ClimberConstants.Climber_Out_P;
    Climber_Out_PIDConfig.kI = ClimberConstants.Climber_Out_I;
    Climber_Out_PIDConfig.kD = ClimberConstants.Climber_Out_D;
    Climber_Out_PIDConfig.kV = ClimberConstants.Climber_Out_F;
    Climber_Ctrl_Config.apply(Climber_Out_PIDConfig);

    Slot0Configs Climber_Back_PIDConfig = new Slot0Configs();
    Climber_Back_PIDConfig.kP = ClimberConstants.Climber_Back_P;
    Climber_Back_PIDConfig.kI = ClimberConstants.Climber_Back_I;
    Climber_Back_PIDConfig.kD = ClimberConstants.Climber_Back_D;
    Climber_Back_PIDConfig.kV = ClimberConstants.Climber_Back_F;
    Climber_Ctrl_Config.apply(Climber_Back_PIDConfig);
    // 設定來回的PID值(依齒輪比決定)

    var Line_Ctrl_Config = climberMotor.getConfigurator();
    Slot0Configs Line_Out_PIDConfig = new Slot0Configs();
    Line_Out_PIDConfig.kP = ClimberConstants.Line_Out_P;
    Line_Out_PIDConfig.kI = ClimberConstants.Line_Out_I;
    Line_Out_PIDConfig.kD = ClimberConstants.Line_Out_D;
    Line_Out_PIDConfig.kV = ClimberConstants.Line_Out_F;
    Line_Ctrl_Config.apply(Line_Out_PIDConfig);

    Slot0Configs Line_Back_PIDConfig = new Slot0Configs();
    Line_Back_PIDConfig.kP = ClimberConstants.Line_Back_P;
    Line_Back_PIDConfig.kI = ClimberConstants.Line_Back_I;
    Line_Back_PIDConfig.kD = ClimberConstants.Line_Back_D;
    Line_Back_PIDConfig.kV = ClimberConstants.Line_Back_F;
    Line_Ctrl_Config.apply(Line_Back_PIDConfig);
    // 設定來回的PID值(依齒輪比決定)
  }

  private final SlewRateLimiter Limiter = new SlewRateLimiter(0.1);
  private final VoltageOut m_request = new VoltageOut(4); // 不知道多少 隨便

  double Climber_Out_targetSpeed = 0.5;
  double smoothedSpeed1 = Limiter.calculate(Climber_Out_targetSpeed);

  double Climber_Back_targetSpeed = -0.5;
  double smoothedSpeed2 = Limiter.calculate(Climber_Back_targetSpeed);

  double Line_Out_targetSpeed = 0.5;
  double smoothedSpeed3 = Limiter.calculate(Line_Out_targetSpeed);

  double Line_Back_targetSpeed = -0.333333;
  double smoothedSpeed4 = Limiter.calculate(Line_Back_targetSpeed);

  public void Climber_Out() {
    Commands.sequence(
        Commands.run(
            () -> {
              climberMotor.setControl(m_request.withOutput(smoothedSpeed1));
            }, this));
  } // 勾住

  public void Climber_Back() {
    Commands.run(
        () -> {
          climberMotor.setControl(m_request.withOutput(smoothedSpeed2));
        }, this);
  } // 放開

  public void Line_Out() {
    Commands.sequence(
        Commands.run(
            () -> {
              climberMotor.setControl(m_request.withOutput(smoothedSpeed3));
            }, this),
        Commands.waitSeconds(2), // 待改
        Commands.run(
            () -> {
              tubeMotor1.set(0);
            }, this));
  }
  // 捲線

  public void Line_Back() {
    Commands.sequence(
        Commands.run(
            () -> {
              tubeMotor1.setControl(m_request.withOutput(smoothedSpeed4));
            }, this),
        Commands.waitSeconds(3), // 暫定
        Commands.run(() -> {
          tubeMotor1.set(0);
        }, this));
  }
  // 放線
  // 慢

}
