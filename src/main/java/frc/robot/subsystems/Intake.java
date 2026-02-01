

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax extensionMotor =  new SparkMax(1,  SparkLowLevel.MotorType.kBrushless);
  private final Timer timer = new Timer();


  public Command intakeAndExtension() {
    timer.reset();
    return Commands.runOnce(() -> {
      if (timer.hasElapsed(3)) {
        for (int i = 0; i < 0.2; i+=0.01){
        intakeMotor.set(i);}
        extensionMotor.set(0);
      } else {
        for (int i = 0; i < 0.2; i+=0.01){
        intakeMotor.set(i);}
        for (int i = 0; i < 0.2; i+=0.01){
        extensionMotor.set(i);}
        
      }
    }, this);
    
  }

  public Command stopIntakeAndBack() {
    timer.reset();
    return Commands.runOnce(() -> {
      if (timer.hasElapsed(3)) {
        intakeMotor.set(0);
        extensionMotor.set(0);
      } 
      else {
        intakeMotor.set(0);
        for (int i = 0; i > -0.2; i-=0.01){
        extensionMotor.set(i);
      }
        
      }
    }, this);

      
  }

  // 之後要依照實際數字更改  
  public Command Autointake() {
    timer.reset();
    return Commands.run(() -> {
      //射8顆球+吸完球
      if (timer.hasElapsed(15)) { //射完8顆加吸完球的時間(準備要射最後一輪)
        intakeMotor.set(0);
        extensionMotor.set(0); 
      } 
      //射8顆球+吸球
      else if (timer.hasElapsed(6)){
        intakeMotor.set(0.5);
        extensionMotor.set(0);
      }
      //射8顆球+準備吸球(讓馬達夠快)
      else if (timer.hasElapsed(5)) { //射完8顆球的時間(要吸第二輪)
        for (int i = 0; i < 0.51; i+=0.01){
        intakeMotor.set(i);}
        extensionMotor.set(0);
      }
      //射8顆球+準備吸球(伸出)
      else if(timer.hasElapsed(3)){
        intakeMoyor.set(0);
        extensionMotor.set(0.4);
      }
      else if (timer.hasElapsed(1)) { //intake伸出來的時間
        intakeMotor.set(i);}
        for (int i = 0.3; i < 0.41; i+=0.01){
        extensionMotor.set(i);
      }
      //射8顆的時間
      else{
        intakeMotor.set(0);
        extensionMotor.set(0.3);
      }
    }, this);
  }

  public Command Auto2cycleintake(){
    timer.reset();
    return Commands.run(() -> {
      //射8顆球+吸球+射球+正在吸球
      if (timer.hasElapsed(15)){ //開始吸第二輪球的時間
        intakeMotor.set(0.5);
        extensionMotor.set(0);
      } 
      //射8顆球+吸球+射球+準備吸球(讓馬達夠快)
      else if (timer.hasElapsed(14)) { //射完8顆球的時間(要吸第二輪)
        for (int i = 0; i < 0.51; i+=0.01){
        intakeMotor.set(i);}
        extensionMotor.set(0);
      }
      //射8顆球+吸球
      else if (timer.hasElapsed(6)){
        intakeMotor.set(0.5);
        extensionMotor.set(0);
      }
      //射8顆球+準備吸球(讓馬達夠快)
      else if (timer.hasElapsed(5)) { //射完8顆球的時間(要吸第二輪)
        for (int i = 0; i < 0.51; i+=0.01){
        intakeMotor.set(i);}
        extensionMotor.set(0);
      }
      //射8顆球+準備吸球(伸出)
      else if(timer.hasElapsed(3)){
        intakeMoyor.set(0);
        extensionMotor.set(0.4);
      }
      else if (timer.hasElapsed(1)) { //intake伸出來的時間
        intakeMotor.set(i);}
        for (int i = 0.3; i < 0.41; i+=0.01){
        extensionMotor.set(i);
      }
      //射8顆的時間
      else{
        intakeMotor.set(0);
        extensionMotor.set(0.3);
      }
    }, this);
  }


  
}