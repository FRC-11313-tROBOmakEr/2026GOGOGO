// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Shooter;


public class AutoRoutines {
        AutoFactory m_factory;

        public AutoRoutines(AutoFactory factory) {
                m_factory = factory;
        }

        public AutoRoutine shoot2cycle(Shooter shoot, Intake autoIntake) {
                final AutoRoutine routine = m_factory.newRoutine("shoot2CycleAuto");
                final AutoTrajectory shoot2cycleAuto = routine.trajectory("shoot2CycleAuto");

                routine.active().onTrue(
                                shoot2cycleAuto.resetOdometry()
                                                .andThen());
                return routine;
        }
}
