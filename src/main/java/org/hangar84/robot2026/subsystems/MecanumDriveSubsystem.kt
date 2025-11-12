package org.hangar84.robot2026.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController.getBatteryVoltage
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.hangar84.robot2026.constants.Constants.Mecanum
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import org.photonvision.PhotonCamera

data object DataTable {

    private val table = NetworkTableInstance.getDefault().getTable("DriveData")

    val frontLeftVelocityEntry: NetworkTableEntry = table.getEntry("Front Left Velocity")
    val frontRightVelocityEntry: NetworkTableEntry = table.getEntry("Front Right Velocity")
    val rearLeftVelocityEntry: NetworkTableEntry = table.getEntry("Rear Left Velocity")
    val rearRightVelocityEntry: NetworkTableEntry = table.getEntry("Rear Right Velocity")

    val frontLeftVoltageEntry: NetworkTableEntry = table.getEntry("Front Left Voltage")
    val frontRightVoltageEntry: NetworkTableEntry = table.getEntry("Front Right Voltage")
    val rearLeftVoltageEntry: NetworkTableEntry = table.getEntry("Rear Left Voltage")
    val rearRightVoltageEntry: NetworkTableEntry = table.getEntry("Rear Right Voltage")
}


class MecanumDriveSubsystem :  Drivetrain() {

    private val rightConfig: SparkMaxConfig = SparkMaxConfig()

    private val frontLeftMotor = SparkMax(Mecanum.FRONT_LEFT_ID, MotorType.kBrushless)
    private val frontRightMotor = SparkMax(Mecanum.FRONT_RIGHT_ID, MotorType.kBrushless)
    private val rearLeftMotor = SparkMax(Mecanum.REAR_LEFT_ID, MotorType.kBrushless)
    private val rearRightMotor = SparkMax(Mecanum.REAR_RIGHT_ID, MotorType.kBrushless)

    private val imu = ADIS16470_IMU()

    private val frontLeftEncoder = frontLeftMotor.absoluteEncoder
    private val frontRightEncoder = frontRightMotor.absoluteEncoder
    private val rearLeftEncoder = rearLeftMotor.absoluteEncoder
    private val rearRightEncoder = rearRightMotor.absoluteEncoder

    var mecanumDrive: MecanumDrive = MecanumDrive(
        frontLeftMotor, rearLeftMotor,
        frontRightMotor, rearRightMotor
    )

    private var frontLeftLocation: Translation2d = Translation2d(0.833, 1.200)
    private var frontRightLocation: Translation2d = Translation2d(0.833, -1.200)
    private var rearLeftLocation: Translation2d = Translation2d(-0.833, 1.200)
    private var rearRightLocation: Translation2d = Translation2d(-0.833, -1.200)

    private val mecanumDriveKinematics: MecanumDriveKinematics = MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation,
        rearLeftLocation, rearRightLocation
    )

    private val mecanumDriveWheelPositions: MecanumDriveWheelPositions = MecanumDriveWheelPositions(
        frontLeftEncoder.position, frontRightEncoder.position,
        rearLeftEncoder.position, rearRightEncoder.position
    )

    private val mecanumDriveOdometry =
        MecanumDriveOdometry(
            mecanumDriveKinematics,
            Rotation2d.fromDegrees(imu.angle),
            mecanumDriveWheelPositions,
            Pose2d()
        )

    private val camera = PhotonCamera("FrontCamera")

    private val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
    private val cameraOffset =
        Transform3d(
            Translation3d(
                Inches.of(-8.0),
                Inches.of(9.0),
                Inches.of(12.0),
            ),
            Rotation3d(0.0, 0.0, 0.0),
        )
    internal val poseEstimator =
        MecanumDrivePoseEstimator(
            mecanumDriveKinematics,
            Rotation2d(Degrees.of(imu.getAngle(imu.yawAxis))),
            mecanumDriveWheelPositions,
            Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1), // State standard deviations
            VecBuilder.fill(1.0, 1.0, 1.0), // Vision standard deviations
        )


    private var frontLeftFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3000, 1.1004)
    private var frontRightFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3000, 1.1004)
    private var rearLeftFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3048, 1.0419)
    private var rearRightFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(4.0, 2.3048, 1.0419)

    private val frontLeftVelocityPIDController: PIDController = PIDController(0.0001, 0.0, 0.0)
    private val frontRightVelocityPIDController: PIDController = PIDController(0.0002, 0.0, 0.0)
    private val rearLeftVelocityPIDController: PIDController = PIDController(0.001, 0.000004, 0.007)
    private val rearRightVelocityPIDController: PIDController = PIDController(0.0013, 0.000004, 0.007)

    val pose: Pose2d
        get() = mecanumDriveOdometry.poseMeters

    private val rotation2d
        get() = Rotation2d.fromDegrees(imu.angle)

    val chassisSpeeds: ChassisSpeeds
        get() =
            mecanumDriveKinematics.toChassisSpeeds(
                MecanumDriveWheelSpeeds(
                    frontLeftEncoder.velocity, frontRightEncoder.velocity,
                    rearLeftEncoder.velocity, rearRightEncoder.velocity
                )
            )

    private val appliedVoltage = Volts.mutable(0.0)

    private val distance = Meters.mutable(0.0)

    private val velocity = MetersPerSecond.mutable(0.0)


    private val identificationRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                // drive =
                { voltage: Measure<VoltageUnit> ->
                    val volts = voltage.`in`(Volts)
                    frontLeftMotor.setVoltage(volts)
                    frontRightMotor.setVoltage(volts)
                    rearLeftMotor.setVoltage(volts)
                    rearRightMotor.setVoltage(volts)
                },
                // log =
                { log: SysIdRoutineLog ->
                    log.motor("drive/front left")
                        .voltage(appliedVoltage.mut_replace(frontLeftMotor.get() * getBatteryVoltage(), Volts))
                        .linearPosition(distance.mut_replace(frontLeftEncoder.position, Meters))
                        .linearVelocity(velocity.mut_replace(frontLeftEncoder.velocity, MetersPerSecond))

                    log.motor("drive/front right")
                        .voltage(appliedVoltage.mut_replace(frontRightMotor.get() * getBatteryVoltage(), Volts))
                        .linearPosition(distance.mut_replace(frontRightEncoder.position, Meters))
                        .linearVelocity(velocity.mut_replace(frontRightEncoder.velocity, MetersPerSecond))

                    log.motor("drive/rear left")
                        .voltage(appliedVoltage.mut_replace(rearLeftMotor.get() * getBatteryVoltage(), Volts))
                        .linearPosition(distance.mut_replace(rearLeftEncoder.position, Meters))
                        .linearVelocity(velocity.mut_replace(rearLeftEncoder.velocity, MetersPerSecond))

                    log.motor("drive/rear right")
                        .voltage(appliedVoltage.mut_replace(rearRightMotor.get() * getBatteryVoltage(), Volts))
                        .linearPosition(distance.mut_replace(rearRightEncoder.position, Meters))
                        .linearVelocity(velocity.mut_replace(rearRightEncoder.velocity, MetersPerSecond))
                },
                // subsystem =
                this,
            ),
        )
    private val DRIVE_FORWARD_COMMAND = run {
        drive(0.0, 0.3, 0.0, false)
    }

    init {
        rightConfig.inverted(false)
        rearRightMotor.configure(rightConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        SmartDashboard.putData("Front Left PID Controller", frontLeftVelocityPIDController)
        SmartDashboard.putData("Front Right PID Controller", frontRightVelocityPIDController)
        SmartDashboard.putData("Rear Left PID Controller", rearLeftVelocityPIDController)
        SmartDashboard.putData("Rear Right PID Controller", rearRightVelocityPIDController)
        SmartDashboard.putData("IMU", imu)
    }

    override fun periodic() {
        mecanumDriveOdometry.update(
            Rotation2d.fromDegrees(imu.angle),
            mecanumDriveWheelPositions
        )

        DataTable.frontLeftVoltageEntry.setDouble(frontLeftMotor.busVoltage)
        DataTable.frontRightVoltageEntry.setDouble(frontRightMotor.busVoltage)
        DataTable.rearLeftVoltageEntry.setDouble(rearLeftMotor.busVoltage)
        DataTable.rearRightVoltageEntry.setDouble(rearRightMotor.busVoltage)

        DataTable.frontLeftVelocityEntry.setDouble(frontLeftEncoder.velocity)
        DataTable.frontRightVelocityEntry.setDouble(frontRightEncoder.velocity)
        DataTable.rearLeftVelocityEntry.setDouble(rearLeftEncoder.velocity)
        DataTable.rearRightVelocityEntry.setDouble(rearRightEncoder.velocity)
    }

    fun resetPose (pose: Pose2d) {
        mecanumDriveOdometry.resetPosition(
            rotation2d, mecanumDriveWheelPositions, pose
        )
    }

    fun driveRelative(relativeSpeeds: ChassisSpeeds) {
        val wheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(relativeSpeeds)

        val frontLeftFed = frontLeftFeedForward.calculate(wheelSpeeds.frontLeftMetersPerSecond)
        val frontRightFed = frontRightFeedForward.calculate(wheelSpeeds.frontRightMetersPerSecond)
        val rearLeftFed = rearLeftFeedForward.calculate(wheelSpeeds.rearLeftMetersPerSecond)
        val rearRightFed = rearRightFeedForward.calculate(wheelSpeeds.rearRightMetersPerSecond)

        val frontLeftOutput = frontLeftVelocityPIDController.calculate(frontLeftEncoder.velocity, wheelSpeeds.frontLeftMetersPerSecond)
        val frontRightOutput = frontRightVelocityPIDController.calculate(frontRightEncoder.velocity, wheelSpeeds.frontRightMetersPerSecond)
        val rearLeftOutput = rearLeftVelocityPIDController.calculate(rearLeftEncoder.velocity, wheelSpeeds.rearLeftMetersPerSecond)
        val rearRightOutput = rearRightVelocityPIDController.calculate(rearRightEncoder.velocity, wheelSpeeds.rearRightMetersPerSecond)

        frontLeftMotor.setVoltage(frontLeftFed + frontLeftOutput)
        frontRightMotor.setVoltage(frontRightFed + frontRightOutput)
        rearLeftMotor.setVoltage(rearLeftFed + rearLeftOutput)
        rearRightMotor.setVoltage(rearRightFed + rearRightOutput)
    }


    override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        mecanumDrive.driveCartesian(ySpeed, xSpeed, rot)
    }

    fun buildAutoChooser(): SendableChooser<Command> {
        AutoBuilder.configure(
            // poseSupplier =
            { poseEstimator.estimatedPosition },
            // resetPose =
            poseEstimator::resetPose,
            // IntelliJ is off its rocker here. The spread operator works here, is practically required, and compiles.
            // The following error should be ignored, since there is no way to remove/hide it.
            // robotRelativeSpeedsSupplier =
            { chassisSpeeds },
            // output =
            this::driveRelative,
            // controller =
            PPHolonomicDriveController(
                // translationConstants =
                PIDConstants(5.0, 0.0, 0.0),
                // rotationConstants =
                PIDConstants(5.0, 0.0, 0.0),
            ),
            // robotConfig =
            RobotConfig.fromGUISettings(),
            // shouldFlipPath =
            { DriverStation.getAlliance()?.get() == DriverStation.Alliance.Red },
            // ...driveRequirements =
            this,
        )

        val autoChooser = AutoBuilder.buildAutoChooser()
        autoChooser.addOption("Leave (Manual)", DRIVE_FORWARD_COMMAND.withTimeout(2.5))

        return autoChooser
    }
}