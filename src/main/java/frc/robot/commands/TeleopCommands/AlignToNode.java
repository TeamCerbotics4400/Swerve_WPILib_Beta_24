// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NodeSelector;

public class AlignToNode extends Command {
  /** Creates a new AlignToNode. */
  DriveTrain m_drive;
  NodeSelector m_selector;
  Joystick joy;

  PIDController angularController;

  private ArrayList<String> cubeNodes;
  
  public AlignToNode(DriveTrain m_drive, NodeSelector m_selector, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_selector = m_selector;
    this.joy = joy;

    cubeNodes = new ArrayList<String>();

    cubeNodes.add("Node 2 Cube");
    cubeNodes.add("Node 5 Cube");
    cubeNodes.add("Node 8 Cube");

    angularController = new PIDController(0, 0, 0, 0);

    angularController.enableContinuousInput(-180, 180);
    angularController.setTolerance(0.05);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new TeleopControl(m_drive, 
    () -> joy.getRawAxis(1), 
    () -> joy.getRawAxis(0), 
    () -> angularController.calculate(LimelightHelpers.getTX(VisionConstants.tagLimelightName)), 
    () -> !joy.getRawButton(4));
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
