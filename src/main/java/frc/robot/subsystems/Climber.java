
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climber extends SubsystemBase {
  private TalonFX climberMotor = new TalonFX(4);
  private TalonFX tubeMotor = new TalonFX(5);
  private Timer timer = new Timer();

  public void Init() {
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    tubeMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  public Command climber() {
    return Commands.runOnce(
        () -> climberMotor.set(0.5), this);
  }

  public Command unClimber() {
    return Commands.runOnce(
        () -> climberMotor.set(-0.5), this);
  }

  public Command rollLine() {
    timer.reset();
    return Commands.runOnce(
        () -> {
          if (timer.hasElapsed(2)) // 暫定
            tubeMotor.set(0.5); // 暫定
        }, this);
    // 捲線(沃)
  }

  public Command unRollLine() {
    timer.reset();
    return Commands.runOnce(
        () -> {
          if (timer.hasElapsed(2)) // 暫定
            tubeMotor.set(-0.5); // 暫定
        }, this);
    // 捲線(沃)
  }
}
