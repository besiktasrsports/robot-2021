// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
  public Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
  public Joystick m_operatorController = new Joystick(JoystickConstants.kOperatorControllerPort);

  public final TurretSubsystem m_turret = new TurretSubsystem();
  public final DriveSubsytem m_robotDrive = new DriveSubsytem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new JoystickDrive(m_robotDrive, () -> -m_driverController.getRawAxis(1),
        () -> m_driverController.getRawAxis(0)));
  }

  private void configureButtonBindings() {

    // Turret Commands
    new JoystickButton(m_driverController, 1).whileHeld(new TurretJoystickCommand(0.3, m_turret));
    new POVButton(m_driverController, 0).whileHeld(new JoystickDrive(m_robotDrive, 1, 1));
    new POVButton(m_driverController, 90).whileHeld(new JoystickDrive(m_robotDrive, forward, rotation));
    new POVButton(m_driverController, 270).whileHeld(new JoystickDrive(m_robotDrive, forward, rotation));
    new POVButton(m_driverController, 360).whileHeld(new JoystickDrive(m_robotDrive, forward, rotation));
    
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
