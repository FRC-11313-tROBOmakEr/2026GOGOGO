package frc.robot.command.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Intake;

public class Intakesuck extends SequentialCommandGroup {

  public Intakesuck (Intake intake) {
    addRequirements(intake);

    addCommands(
     new RunCommand(() ->intake.suck(), intake).withTimeout(3)
    );
  }
}