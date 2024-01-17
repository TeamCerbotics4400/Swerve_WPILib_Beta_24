// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NodeSelector;

public class PathfindAndFollow extends Command {
  /** Creates a new PathfindAndFollow. */
  DriveTrain m_drive;
  NodeSelector m_selector;

  PathConstraints constraints = new PathConstraints
  (AutoConstants.MAX_SpeedMtS,
   AutoConstants.MAX_AccelerationMtS, 
   AutoConstants.MAX_AngularSpeedDegS,
   AutoConstants.MAX_AngularAccelerationDegS);

  public PathfindAndFollow(DriveTrain m_drive, NodeSelector m_selector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_selector = m_selector;

    addRequirements(m_drive, m_selector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.pathfindThenFollowPath(
      m_selector.getPOVToAlign(), 
      constraints, 
      3.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
