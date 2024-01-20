// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class driveForward extends Command {
  /** Creates a new driveForward. */

  // *TODO: Tune these values
  int kPx = 0;
  int kIx = 0;
  int kDx = 0;

  int kPy = 0;
  int kIy = 0;
  int kDy = 0;

  int kPt = 0;
  int kIt = 0;
  int kDt = 0;
  

  public driveForward() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Holonomic drive controller
    var xController = new PIDController(kPx, kIx, kDx);
    var yController = new PIDController(kPy, kIy, kDy);
    // maxvelocity and max acceleration in meters of the robot (need constraints to make a holonomic drive controller)
    Constraints constraints = new TrapezoidProfile.Constraints(3.7, 4.0);
    var thetaController = new ProfiledPIDController(kPt, kIt, kDt, constraints);
    var holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);

    String trajectoryJson = "C:/Users/outre/Documents/Pathweaver/output/Forward.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
    try {
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // *TODO: Create odometry and gyro variables
      odometry.resetPose(trajectory.getInitialPose(), gyro.getRotation2d());
      Pose2d initial = trajectory.getInitialPose(); // (1)
      Pose2d end = Constants.AUTON_START_POSE;
      Transform2d transform = new Transform2d(initial, end);

      trajectory = trajectory.transformBy(transform);
      odometry.resetPose(trajectory.getInitialPose(), gyro.getRotation2d());
      double trajectoryElapsedTime = Timer.get() - trajectoryStartTime;
      Trajectory.State state = trajectory.sample(trajectoryElapsedTime); // (1)

      odometry.update(gyro.getRotation2d(), getModuleStates()); // (2)

      ChassisSpeeds output = holonomicDriveController.calculate(odometry.getPoseMeters(), state, new Rotation2d()); // (3)

      drive(output); // (4)
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
