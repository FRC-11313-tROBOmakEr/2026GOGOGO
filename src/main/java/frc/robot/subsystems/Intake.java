

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

  
}


