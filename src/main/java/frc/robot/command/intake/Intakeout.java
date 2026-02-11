
package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.robot.subsystems.Intake;


public class Intakeout extends SequentialCommandGroup {

  public Intakeout(Intake intake) {

    addCommands(
     new InstantCommand(() ->intake.out(), intake),
     new WaitCommand(0.2),
     new InstantCommand(() ->intake.stopDeploy(), intake), 
     new InstantCommand(() ->intake.suck(), intake)
  
    );
  }
}
