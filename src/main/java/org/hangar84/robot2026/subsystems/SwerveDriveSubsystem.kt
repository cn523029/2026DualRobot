package org.hangar84.robot2026.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import edu.wpi.first.wpilibj2.command.Command
import org.hangar84.robot2026.constants.Constants.Swerve
import org.hangar84.robot2026.swerve.MAXSwerveModule
import org.hangar84.robot2026.swerve.SwerveConfigs
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator


class SwerveDriveSubsystem :  Drivetrain() {
    // Constants

    private val MAX_SPEED = MetersPerSecond.of(4.8)
    private val MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.0)
    private val WHEEL_BASE = Inches.of(24.0)
    private val TRACK_WIDTH = Inches.of(24.5)



    // Create MAXSwerveModules
    val frontLeft: MAXSwerveModule = MAXSwerveModule(
        Swerve.FRONT_LEFT_DRIVING_ID,
        Swerve.FRONT_LEFT_TURNING_ID,
        Degrees.of(270.0),
        SwerveConfigs.drivingConfig,
        SwerveConfigs.turningConfig
    )

    val frontRight: MAXSwerveModule = MAXSwerveModule(
        Swerve.FRONT_RIGHT_DRIVING_ID,
        Swerve.FRONT_RIGHT_TURNING_ID,
        Degrees.of(0.0),
        SwerveConfigs.drivingConfig,
        SwerveConfigs.turningConfig
    )

    val rearLeft: MAXSwerveModule = MAXSwerveModule(
        Swerve.REAR_LEFT_DRIVING_ID,
        Swerve.REAR_LEFT_TURNING_ID,
        Degrees.of(180.0),
        SwerveConfigs.drivingConfig,
        SwerveConfigs.turningConfig
    )

    val rearRight: MAXSwerveModule = MAXSwerveModule(
        Swerve.REAR_RIGHT_DRIVING_ID,
        Swerve.REAR_RIGHT_TURNING_ID,
        Degrees.of(90.0),
        SwerveConfigs.drivingConfig,
        SwerveConfigs.turningConfig
    )

    // The gyro sensor

    private val allModules
        get() = arrayOf(frontLeft, frontRight, rearLeft, rearRight)
    private val allModulePositions: Array<SwerveModulePosition>
        get() = allModules.map { it.position }.toTypedArray()
    internal val allModuleStates: Array<SwerveModuleState>
        get() = allModules.map { it.state }.toTypedArray()
    // -- Sensors --

    private val imu = ADIS16470_IMU()
    private val rotation
        get() = Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ))

    // -- Odometry & Kinematics --

    val kinematics =
        SwerveDriveKinematics(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        )

    internal val odometry = SwerveDriveOdometry(kinematics, rotation, allModulePositions)

    // -- PhotonVision --

    private val camera = PhotonCamera("FrontCamera")

    private val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
    private val cameraOffset =
        Transform3d(
            Translation3d(
                Inches.of(6.0),
                Inches.of(0.0),
                Inches.of(6.0),
            ),
            Rotation3d(0.0, 0.0, 0.0),
        )
    internal val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d(Degrees.of(imu.getAngle(imu.yawAxis))),
            allModulePositions,
            Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1), // State standard deviations
            VecBuilder.fill(1.0, 1.0, 1.0), // Vision standard deviations
        )
    private val photonEstimator =
        PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset)

    private val estimatedRobotPose: EstimatedRobotPose?
        get() {
            var estimate: EstimatedRobotPose? = null
            camera.allUnreadResults.forEach {
                estimate = photonEstimator.update(it).get()
            }

            return estimate
        }

    init {
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve)
    }

    override fun periodic() {
        odometry.update(rotation, allModulePositions)

        if (estimatedRobotPose == null) return

        poseEstimator.update(rotation, allModulePositions)
        poseEstimator.addVisionMeasurement(
            estimatedRobotPose!!.estimatedPose.toPose2d(),
            estimatedRobotPose!!.timestampSeconds,
        )
    }

    // -- Commands --

    override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        val speedX = MAX_SPEED * xSpeed
        val speedY = MAX_SPEED * ySpeed
        val speedAngular = MAX_ANGULAR_SPEED * rot

        val swerveStates =
            kinematics.toSwerveModuleStates(
                if (fieldRelative) {
                    ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedAngular, rotation)
                } else {
                    ChassisSpeeds(speedX, speedY, speedAngular)
                },
            )

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_SPEED)

        allModules.forEachIndexed { i, module -> module.desiredState = swerveStates[i] }
    }

    fun driveRelative(chassisSpeeds: ChassisSpeeds) {
        val desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds)

        allModules.forEachIndexed { i, module -> module.desiredState = desiredStates[i] }
    }

    fun park(): Command =
        this.run {
            val positions =
                arrayOf(
                    Rotation2d.fromDegrees(45.0), // Front left
                    Rotation2d.fromDegrees(-45.0), // Front right
                    Rotation2d.fromDegrees(-45.0), // Rear left
                    Rotation2d.fromDegrees(45.0), // Rear right
                )

            allModules.forEachIndexed { i, module -> module.desiredState = SwerveModuleState(0.0, positions[i]) }
        }
}
