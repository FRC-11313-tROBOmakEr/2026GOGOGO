
package frc.robot.command.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;

public class Intakein extends SequentialCommandGroup {
  
  public Intakein(Intake intake) {

    addCommands(
     new RunCommand(() ->intake.back(), intake).withTimeout(3),
     new InstantCommand(() ->intake.stopDeploy(), intake),
     new InstantCommand(() ->intake.stopRoller(), intake));
  }
}

