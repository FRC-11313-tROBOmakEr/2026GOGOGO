
package frc.robot.command.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Intake;


public class Intakeout extends SequentialCommandGroup {

  public Intakeout(Intake intake) {

    addCommands(
     new RunCommand(() ->intake.out(), intake).withTimeout(3),
     new RunCommand(() ->intake.stopDeploy(), intake), 
     new RunCommand(() ->intake.suck(), intake)
    );
  }
}
