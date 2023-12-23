// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoCommands.IdleArm;
import frc.robot.commands.TeleopCommands.TeleopControl;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;
import team4400.StateMachines;
import team4400.StateMachines.IntakeState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveTrain m_drive = new DriveTrain();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  
  private final Joystick chassisDriver = new Joystick(0);
  private final Joystick subsystemsDriver = new Joystick(1);

  private final NodeSelector m_selector = new NodeSelector(subsystemsDriver);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("ArmIdle", new IdleArm(m_arm, m_wrist));
    NamedCommands.registerCommand("SelectLow", 
                            new InstantCommand(() -> m_selector.selectLevel(0)));
    NamedCommands.registerCommand("SelectMid", 
                            new InstantCommand(() -> m_selector.selectLevel(1)));
    NamedCommands.registerCommand("SelectHigh", 
                            new InstantCommand(() -> m_selector.selectLevel(2)));
    NamedCommands.registerCommand("SelectMaria", 
                            new InstantCommand(() -> m_selector.selectLevel(3)));

    m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> chassisDriver.getRawAxis(1), 
    () -> chassisDriver.getRawAxis(0), 
    () -> -chassisDriver.getRawAxis(4), 
    () -> !chassisDriver.getRawButton(4)));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(chassisDriver, 1).onTrue(
      new InstantCommand(() -> m_drive.zeroHeading()));

    //Left bumper
    new JoystickButton(chassisDriver, 5)
   .onTrue(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.RIGHT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

  //Right bumper
  new JoystickButton(chassisDriver, 6)
  .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
  .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
  .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
  .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

  //Controller 2
   //Pov up
   new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_selector.updateSelectionUp()));

   //Pov down
   new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_selector.updateSelectionDown()));

   //Left bumper
   
    new JoystickButton(subsystemsDriver, 5)
   .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   //Right bumper
   new JoystickButton(subsystemsDriver, 6)
   .onTrue(m_arm.goToPosition(ArmConstants.SCORING_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 2).toggleOnTrue(
    new InstantCommand(() -> StateMachines.setIntakeIdle()));

   /**********  DEBUGGING  **********/
   //new JoystickButton(chassisDriver, 1).whileTrue(new DebugShooterCommand(m_shooter));

   //new JoystickButton(chassisDriver, 2).whileTrue(new PIDModuleTuner(m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("ThreePiece");
  }

  public DriveTrain getDrive(){
    return m_drive;
  }
}
