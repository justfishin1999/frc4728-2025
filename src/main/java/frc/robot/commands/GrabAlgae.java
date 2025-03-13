// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAlgae extends SequentialCommandGroup {
  /** Creates a new ScoreL2_3. */
  public GrabAlgae(Elevator m_Elevator, Wrist m_Wrist, Intake m_Intake) {
    super(
    new SetElevatorGrabAlgae(m_Elevator),
    new WaitCommand(0.5),
    new SetWristGrabAlgae(m_Wrist),
    new WaitCommand(0.5),
    new RunIntakeIn (m_Intake).withTimeout(15));
  }
}
