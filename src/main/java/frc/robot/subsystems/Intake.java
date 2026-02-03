package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeContants;

public class Intake extends SubsystemBase {

  private final SparkMax Intake_Roller = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax Intake_Ctrl = new SparkMax(2, SparkLowLevel.MotorType.kBrushless); 
  //控制intake伸縮的馬達

  private final Timer timer = new Timer();

  private final SlewRateLimiter intakeLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter extensionLimiter = new SlewRateLimiter(0.5);

  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    SparkMaxConfig extensionConfig = new SparkMaxConfig();

    intakeConfig.smartCurrentLimit(40)
        .idleMode(SparkMaxConfig.IdleMode.kBrake);

    intakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1).i(0).d(0);

    extensionConfig.smartCurrentLimit(40)
        .idleMode(SparkMaxConfig.IdleMode.kBrake);

    Intake_Roller.configure(intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    Intake_Ctrl.configure(extensionConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // 指令:吸球和伸出
  public Command intakeAndExtension() {
    return Commands.sequence(
      Commands.run(() -> {//伸出
        Intake_Roller.set(0.5);
        Intake_Ctrl.set(0.5);
      }, this),
      Commands.waitSeconds(3),//伸出需要時間
      Commands.run(() -> {//Ctrl停止,吸球
        Intake_Roller.set(0.5);
        Intake_Ctrl.set(0);
      }, this)
    );
  }

  // 指令:停止吸球和收回
  public Command stopIntakeAndBack() {
    return Commands.sequence(
      Commands.run(() -> {//收回
        Intake_Roller.set(0);
        Intake_Ctrl.set(-0.5);
      }, this),
      Commands.waitSeconds(3),//收回需要時間
      Commands.run(() -> {//全部停止
        Intake_Roller.set(0);
        Intake_Ctrl.set(0);
      }, this)
    );

  }

  // Auto
  public Command autorunIntake(double speed) {
    return Commands.run(() -> Intake_Roller.set(intakeLimiter.calculate(speed)), this);
  }

  public Command autorunExtension(double speed) {
    return Commands.run(() -> Intake_Ctrl.set(extensionLimiter.calculate(speed)), this);
  }

  public Command autostopAll() {
    return Commands.runOnce(() -> {
      Intake_Roller.stopMotor();
      Intake_Ctrl.stopMotor();
      intakeLimiter.reset(0);
      extensionLimiter.reset(0);
    }, this);
  }

  public Command AutointakeExtension() {
    return Commands.sequence(
        Commands.parallel(
            autorunIntake(0.2),
            autorunExtension(0.2)).withTimeout(3.0),
        autorunIntake(0.2).alongWith(autorunExtension(0)));
  }

  // 依照實際數值更改
  public Command autointake1() {
    return Commands.sequence(
        autorunExtension(0.4).withTimeout(1.0), // 0-1秒:伸出(馬達加速)
        autorunExtension(0.4).withTimeout(3.0), // 1-3秒: 伸出
        autorunIntake(0.5).withTimeout(2.0), // 3-5秒: 開始吸球 (馬達加速)
        autorunIntake(0.5).withTimeout(3.0), // 5-8秒: 持續吸球
        autostopAll() // 8秒後停止
    );
  }

  public Command autointake2() {
    return Commands.sequence(
        autorunIntake(0.5).withTimeout(2.0), // 0~2秒: 開始吸球 (馬達加速)
        autorunIntake(0.5).withTimeout(3.0), // 2-5秒: 持續吸球
        autostopAll() // 5秒後停止
    );
  }
}

// public Command auto2cycleintake() {
// return Commands.sequence(
// autorunExtension(0.4).withTimeout(3.0), // 0-1秒:伸出(馬達加速)
// autorunExtension(0.4).withTimeout(2.0), // 1~3秒:伸出
// autorunIntake(0.5).withTimeout(1.0), //3-5秒:開始吸球(馬達加速)
// autorunIntake(0.5).withTimeout(8.0), //5-8秒:持續吸球
// autorunIntake(0.5).withTimeout(1.0), // 6-14秒
// autostopAll() // 14-15秒
// );
// }
// }
