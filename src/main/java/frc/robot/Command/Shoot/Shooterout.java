
package frc.robot.Command.Shoot;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;



public class Shooterout extends SequentialCommandGroup {

  public Shooterout(Shooter shooter) {
    addRequirements(shooter);
    addCommands(
    new InstantCommand(() -> shooter.Shooter_Out(),shooter),
    new InstantCommand(() -> shooter.angle_out(),shooter),
    Commands.waitSeconds(0.8),
    new InstantCommand(() -> shooter.InxererWorking(),shooter)
    );
  
  
  }
}
