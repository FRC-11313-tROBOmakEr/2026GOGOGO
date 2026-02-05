
package frc.robot.Command.Shoot;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;


public  class Shootin  extends SequentialCommandGroup{

  public Shootin(Shooter shooter) {  
    addCommands(  
    new RunCommand(() -> shooter.Shooter_Stop(), shooter),
    new InstantCommand(() -> shooter.angle_in(), shooter),
    new RunCommand(() -> shooter.IndexerStop(), shooter));
   
    }}