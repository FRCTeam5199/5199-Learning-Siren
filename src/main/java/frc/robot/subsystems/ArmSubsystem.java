// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.abstractMotorInterfaces.VortexMotorController;

public class ArmSubsystem extends SubsystemBase {
  VortexMotorController vortexMotorController;

  PIDController vortexPIDController;

  private double setpoint;

  /** Creates a new ExamplePIDControlledMotor. */
  public ArmSubsystem() {
    vortexMotorController = new VortexMotorController(Constants.VORTEX_MOTOR_CONTROLLER_ID);
    vortexPIDController = new PIDController(0.01, 0, 0);
    setpoint = 0;
  }

  public Command setArmSetpoint(double setpoint) {
    return runOnce(() -> this.setpoint = setpoint);
  }

  public Command setArmSpeakerSetpoint() {
    return runOnce(() -> this.setpoint = Constants.Setpoints.SPEAKER_SETPOINT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    vortexMotorController.set(vortexPIDController.calculate(vortexMotorController.getRotations(), setpoint));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
