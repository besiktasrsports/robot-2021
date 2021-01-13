// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.hooper.HoopperCommand;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.subsystems.HoopperSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
  public Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
  public Joystick m_operatorController = new Joystick(JoystickConstants.kOperatorControllerPort);
  private final HoopperSubsystem m_hoopper = new HoopperSubsystem();
  public final TurretSubsystem m_turret = new TurretSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Turret Commands
    new JoystickButton(m_driverController, 1).whileHeld(new TurretJoystickCommand(0.3, m_turret));
    new JoystickButton(m_driverController, 2).whileHeld(new HoopperCommand(m_hoopper, 0.5));
    new JoystickButton(m_driverController, 3).whileHeld(new HoopperCommand(m_hoopper, -0.5));
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
