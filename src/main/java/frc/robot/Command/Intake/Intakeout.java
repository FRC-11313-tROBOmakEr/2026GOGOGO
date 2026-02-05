

package frc.robot.Command.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;


public class Intakeout extends SequentialCommandGroup {

  public Intakeout(Intake intake) {

    addCommands(
     new InstantCommand(()->intake.intakeOut(), intake).withTimeout(3),
     new InstantCommand(()->intake.intake_dont_do_that(), intake), 
     new InstantCommand(()->intake.suck(), intake)
    );
  }
}
