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
      new InstantCommand(() ->shooter.angle_out()),
      new RunCommand(() ->shooter.Shooter_Out(),shooter),
      new RunCommand(() ->shooter.IndexrWorking(), shooter).withTimeout(4.2),
      new RunCommand(() ->intake.intakeOut(),intake).withTimeout(5),
      new RunCommand(() ->intake.intake_dont_do_that(), intake), 
      new RunCommand(() ->intake.suck(), intake),
      new RunCommand(() ->shooter.Shooter_Out(),shooter).withTimeout(5),
      new RunCommand(() ->intake.intake_dont_do_that(),intake));
  }   

  }
