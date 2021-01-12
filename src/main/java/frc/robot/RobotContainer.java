// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleCompressor;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
  public Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
  public Joystick m_operatorController = new Joystick(JoystickConstants.kOperatorControllerPort);

  public final TurretSubsystem m_turret = new TurretSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Turret Commands
    new JoystickButton(m_driverController, 1).whileHeld(new TurretJoystickCommand(0.3, m_turret));
    
    // Intake Commands
    new JoystickButton(m_driverController, 2).whileHeld(new RunIntake(m_intake, 0.5));
    new JoystickButton(m_operatorController, 1).whenPressed(new ToggleIntake(m_intake));
    new JoystickButton(m_operatorController, 3).whileHeld(new ToggleCompressor(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
