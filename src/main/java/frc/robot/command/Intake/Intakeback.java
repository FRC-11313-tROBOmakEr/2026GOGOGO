
package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;

public class Intakeback extends SequentialCommandGroup {
  
  public Intakeback(Intake intake) {

    addCommands(
     new RunCommand(() ->intake.back(), intake).withTimeout(3),
     new InstantCommand(() ->intake.stopDeploy(), intake),
     new InstantCommand(() ->intake.stopRoller(), intake));
  }
}

