
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotor_ID ,"canbus");
  private TalonFX tubeMotor1 = new TalonFX(ClimberConstants.tubeMotor1_ID ,"canbus");
  private TalonFX tubeMotor2 = new TalonFX(ClimberConstants.tubeMotor2_ID ,"canbus");
  private Timer timer = new Timer();
  private Follower follower ;

  public void Init() {
    //var talonFXConfigs = new TalonFXConfiguration();
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor1.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor2.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor2.setControl(follower.withLeaderID(1));
    //(可能要反轉)config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
    //climberMotor.setPosition(0);
    //回歸原始設定         
    //climberMotor.setPosition(0);
    //回歸原始設定  

    var Climber_Ctrl_Config = climberMotor.getConfigurator();
    Slot0Configs Climber_Out_PIDConfig =new Slot0Configs();
        Climber_Out_PIDConfig.kP = ClimberConstants.Climber_Out_P;
        Climber_Out_PIDConfig.kI = ClimberConstants.Climber_Out_I;
        Climber_Out_PIDConfig.kD = ClimberConstants.Climber_Out_D;
        Climber_Out_PIDConfig.kV = ClimberConstants.Climber_Out_F;
        Climber_Ctrl_Config.apply(Climber_Out_PIDConfig);

        Slot0Configs Climber_Back_PIDConfig =new Slot0Configs();
        Climber_Back_PIDConfig.kP = ClimberConstants.Climber_Back_P;
        Climber_Back_PIDConfig.kI = ClimberConstants.Climber_Back_I;
        Climber_Back_PIDConfig.kD = ClimberConstants.Climber_Back_D;
        Climber_Back_PIDConfig.kV = ClimberConstants.Climber_Back_F;
        Climber_Ctrl_Config.apply(Climber_Back_PIDConfig);
        //設定來回的PID值(依齒輪比決定)

    
    
    var Line_Ctrl_Config = climberMotor.getConfigurator();
    Slot0Configs Line_Out_PIDConfig =new Slot0Configs();
        Line_Out_PIDConfig.kP = ClimberConstants.Line_Out_P;
        Line_Out_PIDConfig.kI = ClimberConstants.Line_Out_I;
        Line_Out_PIDConfig.kD = ClimberConstants.Line_Out_D;
        Line_Out_PIDConfig.kV = ClimberConstants.Line_Out_F;
        Line_Ctrl_Config.apply(Line_Out_PIDConfig);

        Slot0Configs Line_Back_PIDConfig =new Slot0Configs();
        Line_Back_PIDConfig.kP = ClimberConstants.Line_Back_P;
        Line_Back_PIDConfig.kI = ClimberConstants.Line_Back_I;
        Line_Back_PIDConfig.kD = ClimberConstants.Line_Back_D;
        Line_Back_PIDConfig.kV = ClimberConstants.Line_Back_F;
        Line_Ctrl_Config.apply(Line_Back_PIDConfig);
        //設定來回的PID值(依齒輪比決定)
  }

  public Command Climber_Out() {
    return Commands.runOnce(
        () -> climberMotor.set(0.5), this);
  }//勾住

  public Command Climber_Back() {
    return Commands.runOnce(
        () -> climberMotor.set(-0.5), this);
  }//放開


  public Command Line_Out() {
    timer.reset();
    timer.start();

    return Commands.runOnce(
        () -> {
          if (timer.hasElapsed(2)){  // 暫定
            tubeMotor1.set(0); // 暫定
          }
          else {
            tubeMotor1.set(0.5);
          }
        },this);
       // 捲線
  }

  public Command Line_back() {
    timer.reset();
    timer.start();

    return Commands.runOnce(
        () -> {
          if (timer.hasElapsed(2)){  // 暫定
            tubeMotor1.set(-0.3); // 暫定
          }
          else {
          tubeMotor2.set(-0.5);
          }
        }, this);
      // 放線
      //慢
  }
}
