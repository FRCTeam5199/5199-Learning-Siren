// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.abstractMotorInterfaces.VortexMotorController;

public class ExamplePercentOutputMotor extends SubsystemBase {
  VortexMotorController vortexMotorController;
  /** Creates a new ExampleSubsystem. */
  public ExamplePercentOutputMotor() {
    vortexMotorController = new VortexMotorController(Constants.VORTEX_MOTOR_CONTROLLER_ID);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command examplePercentOutputCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> vortexMotorController.set(0.5));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
