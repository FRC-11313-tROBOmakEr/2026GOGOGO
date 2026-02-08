
package frc.robot.command.Shoot;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public  class Shootin  extends SequentialCommandGroup{

  public Shootin(Shooter shooter) {  
    addCommands(  
    new InstantCommand(() -> shooter.stopFlyWheels(), shooter),
    new InstantCommand(() -> shooter.angle_in(), shooter),
    new InstantCommand(() -> shooter.stopConveyor(), shooter));
   
    }}