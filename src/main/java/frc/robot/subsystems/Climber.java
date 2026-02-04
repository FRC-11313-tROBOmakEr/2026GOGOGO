
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
//import com.revrobotics.spark.config.ClosedLoopConfig;

//import com.ctre.phoenix6.controls.VoltageOut;

public class Climber extends SubsystemBase {

  private TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotor_ID, "canbus");
  private TalonFX tubeMotor1 = new TalonFX(ClimberConstants.tubeMotor1_ID, "canbus");
  private TalonFX tubeMotor2 = new TalonFX(ClimberConstants.tubeMotor2_ID, "canbus");
  private Follower follower;
  TalonFXConfiguration configuration = new TalonFXConfiguration();

  public Climber() {
    // climberMotor.setPosition(0);
    // 回歸原始設定
    // climberMotor.setPosition(0);
    // 回歸原始設定

    // var talonFXConfigs = new TalonFXConfiguration();
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor1.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor2.getConfigurator().apply(configuration);
    tubeMotor2.setControl(follower.withLeaderID(1));
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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

    if (DriverStation.isEnabled()) {
      climberMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_StartUp));
    }
  }

  public void Climb_Zero() {
    climberMotor.setControl(new MotionMagicVelocityVoltage(ClimberConstants.Climb_Zero));
    tubeMotor1.setControl(new MotionMagicVelocityVoltage(ClimberConstants.Climb_Zero));
  }

  public void Climb() {
    climberMotor.setControl(new MotionMagicVelocityVoltage(ClimberConstants.Climb_Angle));
  }

  public void Climber_up() {
    climberMotor.set(0.5);
  }

  public void Line_up() {
    tubeMotor1.set(0.5);
  }

  public void Climber_down() {
    climberMotor.set(-0.5);
  }

  public void Line_down() {
    tubeMotor1.set(-0.5);
  }

  public void Stop() {
    climberMotor.set(0);
    tubeMotor1.set(0);
  }

  public void StopAll() {
    climberMotor.stopMotor();
    tubeMotor1.stopMotor();
  }

  // Command
  public void climbing() {
    // 要放command
  }
}
