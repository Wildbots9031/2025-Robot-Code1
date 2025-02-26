// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class climbPosition extends Command {
  private final ClimbSubsystem m_Climb;
  /** Creates a new Climb. */
  public climbPosition(ClimbSubsystem climbPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.m_Climb = climbPosition;
  addRequirements(m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Climb.climb_position();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climb.climb_position();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
