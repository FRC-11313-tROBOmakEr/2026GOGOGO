// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.ctre.phoenix6.hardware.CANcoder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intake extends Command {
  private final SparkMax intake = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax intakeForward =  new SparkMax(1,  SparkLowLevel.MotorType.kBrushless); 
  
  public intake() {
     intake.set(0.5); 
     intakeForward.set(0.2);
  }
  public Command intakeBack(){
     intake.set(0);
     intakeForward.set(-0.2);
     return null ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
