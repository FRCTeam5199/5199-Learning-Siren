// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//All subsystems are commented out because they don't actually exist yet
public final class Autos {
  //Initialize subsytems here

  //Example of initialization:
  // public ShooterSubsystem shooter;
  // public IntakeSubsystem intake;
  // public IndexerSubsystem indexer;
  public CommandSwerveDrivetrainerveDrive swerveDrive;


  SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(3, .01, 0), new PIDConstants(1.7, .06, 0.00), 5, .21, new ReplanningConfig());

  //Configure with your own subsystems
  //Pass in instances of the subsystems from RobotContainer
  public Autos(/*ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, */ CommandSwerveDrivetraineDrive swerveDrive) { //Example parameters
    //Example setting:
    // this.shooter = shooter;
    // this.intake = intake;
    // this.indexer = indexer;
    // this.swerveDrive = swerveDrive;

    AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, () -> false, swerveDrive);

    //Add your own commands here
    NamedCommands.registerCommand("placeholder", /*shooter.runShooterAtPercent(.5)*/ null);
    NamedCommands.registerCommand("placeholderCommandGroup", new SequentialCommandGroup(
            // shooter.runShooterAtPercent(0.5),
            // intake.deployIntake().withTimeout(0.2)
    ));
  }

  //Add methods calling your autons, make sure names are exactly the same
  public Command testAuton() {
    return AutoBuilder.buildAuto("placeholder");
  }
}
