
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class PoseEstimation {
    private final SwerveDrivePoseEstimator poseEstimator;

    private final LimelightBackend[] backends;
    private final boolean[] backendToggles;

    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(2);

    private static final double DIFFERENTIATION_TIME = Robot.defaultPeriodSecs;

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.DRIVE_KINEMATICS,
                RobotContainer.drivetrain.getHeadingRotation2d(),
                RobotContainer.drivetrain.getModulePositions(),
                new Pose2d(),
                SwerveConstants.ODOMETRY_STD_DEV,
                VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );

        backends = new LimelightBackend[1];
        backendToggles = new boolean[1];

        // try {
        //     backends[0] = new PhotonVisionBackend("back-camera", VisionConstants.ROBOT_TO_SHOOTER_LL);
        //     backendToggles[0] = true;
        // } catch (Exception e) {
        //     System.out.println("Failed to initialize PhotonVision");
        //     e.printStackTrace();
        // }

        backends[0] = new LimelightBackend();
        backendToggles[0] = true;
    }

    public void periodic() {
        for (int i = 0; i < backends.length; i++) {
            if (backendToggles[i]) {
                // this is a hack to get around an issue in `SwerveDrivePoseEstimator`
                // where two measurements cannot share the same timestamp
                double timestampOffset = 1e-9 * i;

                backends[i].getMeasurement().map((measurement) -> {
                    measurement.timestamp += timestampOffset;
                    return measurement;
                }).ifPresent(this::addVisionMeasurement);
            }
        }

        poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
    }

    public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = poseHistory.getSample(now).orElseGet(Pose2d::new).getTranslation();
        Translation2d previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).orElseGet(Pose2d::new).getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.getHeadingRotation2d(), RobotContainer.drivetrain.getModulePositions(), pose);
    }

    private void addVisionMeasurement(VisionBackend.Measurement measurement) {
        poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
    }

    public void setShooterLL(boolean on){
        backendToggles[0] = on;
    }
}