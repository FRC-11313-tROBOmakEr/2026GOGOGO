
package frc.robot.Command.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Intake;

public class Intakein extends SequentialCommandGroup {
  
  public Intakein(Intake intake) {

    addCommands(

     new RunCommand(() ->intake.intakeBack(), intake).withTimeout(3),
     new RunCommand(() ->intake.intake_dont_do_that(), intake),
     new RunCommand(() ->intake.shoot(), intake));
  }
}

