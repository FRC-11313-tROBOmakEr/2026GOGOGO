// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax extensionMotor =  new SparkMax(1,  SparkLowLevel.MotorType.kBrushless);
  private final Timer timer = new Timer();
  int extensionTime = 3;

  
  public Intake() {
    timer.reset();
  } //機器人開始運動，timer歸零

  public void extensionAndIntake() {
      extensionMotor.set(0.2); //伸出要比縮回慢一點點，避免衝突
    if(timer.hasElapsed(extensionTime)){
      extensionMotor.stopMotor();; // 到位停止
    }
    intakeMotor.set(0.5); // 持續吸球
  }
      
  public void backAndStopIntake() {
    if (timer.hasElapsed(extensionTime)) { 
      extensionMotor.set(-0.3);
    } else {
      extensionMotor.stopMotor();; // 到位停止
    }
    intakeMotor.set(0); // 停止吸球
  }

  public void stopAll() {
    extensionMotor.set(0);
    intakeMotor.set(0);
  }

  // 方便在 RobotContainer 檢查是否到位的判斷式
  public boolean isRetracted() {
    return timer.advanceIfElapsed(extensionTime);
  }
}


