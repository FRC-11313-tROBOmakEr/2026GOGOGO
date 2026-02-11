package frc.robot.command.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
public class Leftshoot2cycle extends SequentialCommandGroup {

  public Leftshoot2cycle (Shooter shooter, Intake intake) {

    addRequirements(shooter);
    addRequirements(intake);

    addCommands(
      new InstantCommand(() ->intake.out(),intake),
      new InstantCommand(() ->shooter.Shooter_Out(),shooter),
      new InstantCommand(() ->shooter.indexerRun(), shooter).withTimeout(4.2),
      new InstantCommand(() ->intake.out(),intake).withTimeout(5),
      new InstantCommand(() ->intake.stopDeploy(), intake), 
      new InstantCommand(() ->intake.suck(), intake),
      new InstantCommand(() ->shooter.Shooter_Out(),shooter).withTimeout(5),
      new InstantCommand(() ->intake.stopDeploy(),intake));
  }   

  }
