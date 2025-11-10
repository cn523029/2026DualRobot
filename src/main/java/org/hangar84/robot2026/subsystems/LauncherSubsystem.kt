package org.hangar84.robot2026.subsystems

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.hangar84.robot2026.constants.Constants

object LauncherSubsystem : Subsystem {

    val LauncherMotor = SparkMax(Constants.LauncherSubsystem.LauncherMotor, MotorType.kBrushless)
    private val FollowerLauncherMotor = SparkMax(Constants.LauncherSubsystem.FollowerLauncherMotor, MotorType.kBrushless)

    private val followerConfig = SparkMaxConfig()
    private val launcherConfig = SparkMaxConfig()

    val LAUNCH: Command
        get() =
            runOnce {
                LauncherMotor.set(1.0)
            }
                .withTimeout(1.0)
                .andThen({
                    LauncherMotor.set(1.0)
                    FollowerLauncherMotor.set(1.0)
                })
                .withTimeout(1.0)
                .andThen({
                    LauncherMotor.set(0.0)
                    FollowerLauncherMotor.set(0.0)
                })

    val INTAKE: Command
        get() =
            runOnce {
                LauncherMotor.set(-1.0)
                FollowerLauncherMotor.set(-1.0)
            }
                .withTimeout(1.0)
                .andThen({
                    LauncherMotor.set(0.0)
                    FollowerLauncherMotor.set(0.0)
                })
    val LAUNCH_FAST: Command
        get() =
            runOnce {
                LauncherMotor.set(-1.0)
                FollowerLauncherMotor.set(1.0)
            }
                .withTimeout(1.0)
                .andThen({
                    LauncherMotor.set(-1.0)
                    FollowerLauncherMotor.set(0.0)
                })
                .withTimeout(1.0)
                .andThen({
                    LauncherMotor.set(1.0)
                    FollowerLauncherMotor.set(1.0)
                })

    init {
        LauncherMotor.configure(
            launcherConfig.inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
    }
}