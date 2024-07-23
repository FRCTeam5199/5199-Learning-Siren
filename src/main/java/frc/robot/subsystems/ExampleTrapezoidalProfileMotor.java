// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A robot arm subsystem that moves with a motion profile. */
public class ExampleTrapezoidalProfileMotor extends SubsystemBase {
  /** Create a new ExampleTrapezoidalProfileSubsystem. */
  private TalonFX motor;
  private TalonFXConfiguration talonFXConfiguration;
  private PositionVoltage positionVoltage;
  private TrapezoidProfile trapezoidProfile;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State state;
  private static ExampleTrapezoidalProfileMotor exampleSubsytem;

  private ExampleTrapezoidalProfileMotor() {}

  public static ExampleTrapezoidalProfileMotor getInstance() {
    if (exampleSubsytem == null) {
      exampleSubsytem = new ExampleTrapezoidalProfileMotor();
    }
    return exampleSubsytem;
  }

  public void init() {
    motor = new TalonFX(0);

    talonFXConfiguration = new TalonFXConfiguration();
    //Add configs here

    motor.getConfigurator().apply(talonFXConfiguration);
    
    //Config PID Values and add kV and kS for VelocityVoltage control
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    motor.getConfigurator().apply(slot0Configs);

    //withEnableFoc increases power of the motors
    //withFeedForward is the default voltage to apply, this counteracts gravity for an arm.
    positionVoltage = new PositionVoltage(0).withEnableFOC(true).withFeedForward(1);

    trapezoidProfile = new TrapezoidProfile(
      //Units are the same as the control method, PositionVoltage and VelocityVoltage is rps
      new TrapezoidProfile.Constraints(0, 0)
    );

    state = new TrapezoidProfile.State();
    //if you want the motor to move to a state on initialaztion, add the setpoint parameter and then 0 for the velocity paramter
    goal = new TrapezoidProfile.State();
  }

  @Override
  public void periodic() {
    //0.02 is the time in between periodic calls
    state = trapezoidProfile.calculate(0.02, state, goal);
    positionVoltage.Position = state.position;
    positionVoltage.Velocity = state.velocity;
    motor.setControl(positionVoltage);
  }

  public Command setTrapezoidalSetpoint(double position) {
    //Always use 0 for the velocity parameter because you don't want the motor to be moving when it gets to the desired setpoint
    return this.runOnce(() -> goal = new TrapezoidProfile.State(position, 0));
  }
}