// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.abstractMotorInterfaces.VortexMotorController;

public class ExampleSubsystem extends SubsystemBase {
  private final VortexMotorController test_vortex_motor_controller;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    test_vortex_motor_controller = new VortexMotorController(Constants.VORTEX_MOTOR_CONTROLLER_ID);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          test_vortex_motor_controller.set(0.5);
        });
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   * 
   * 
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some oolean state, such as a digital sensor
    return false;
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
