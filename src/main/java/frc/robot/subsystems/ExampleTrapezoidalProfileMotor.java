// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.abstractMotorInterfaces.VortexMotorController;

/** A robot arm subsystem that moves with a motion profile. */
public class ExampleTrapezoidalProfileMotor extends TrapezoidProfileSubsystem {
  private final VortexMotorController vortexMotorController = new VortexMotorController(Constants.VORTEX_MOTOR_CONTROLLER_ID);
  private final PIDController vortexPIDController = new PIDController(0.01, 0, 0);
  /** Create a new ExampleTrapezoidalProfileArmMotor TrapezoidProfileSubsystem. */
  public ExampleTrapezoidalProfileMotor() {
    super(new TrapezoidProfile.Constraints(3, 10), 0.5);
    vortexPIDController.setPID(0.01, 0, 0);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) { 
    vortexMotorController.set(vortexPIDController.calculate(vortexMotorController.getRotations(), setpoint.position));
  }

  public Command setGoalCommand(double koffsetRads) {
    return Commands.runOnce(() -> setGoal(koffsetRads), this);
  }
}