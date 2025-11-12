package org.hangar84.robot2026

//import com.pathplanner.lib.auto.AutoBuilder
//import com.pathplanner.lib.config.PIDConstants
//import com.pathplanner.lib.config.RobotConfig
//import com.pathplanner.lib.controllers.PPHolonomicDriveController
//import edu.wpi.first.math.MathUtil
//import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.constants.RobotType
import org.hangar84.robot2026.subsystems.*
import org.hangar84.robot2026.commands.DriveCommand


/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
object RobotContainer {
    private val buttonA = DigitalInput(9)

    private val robotType: RobotType
        get() = if (buttonA.get()) {
            RobotType.SWERVE
        } else {
            RobotType.MECANUM
        }

    // The robot's subsystems
    private val drivetrain: Drivetrain
        get() = when (robotType) {
            RobotType.SWERVE -> SwerveDriveSubsystem()
            RobotType.MECANUM -> MecanumDriveSubsystem()
            else -> SwerveDriveSubsystem()
        }
    // The driver's controller
    private val controller: CommandXboxController = CommandXboxController(0)
    private var autoChooser: SendableChooser<Command> = SwerveDriveSubsystem().buildAutoChooser()

    val autonomousCommand: Command
        get() = autoChooser.selected ?: InstantCommand()

    init {
        SmartDashboard.putString("Selected Robot Type", robotType.name)
        SmartDashboard.putData("Auto Chooser", autoChooser)
        updateAutoChooser()

        configureBindings()
    }
    private fun updateAutoChooser() {
        autoChooser = when (robotType) {
            RobotType.SWERVE -> SwerveDriveSubsystem().buildAutoChooser()
            RobotType.MECANUM -> MecanumDriveSubsystem().buildAutoChooser()
        }
    }
    private fun configureBindings() {
        drivetrain.defaultCommand = DriveCommand(
            drivetrain,
            { -controller.leftY },
            { controller.leftX },
            { controller.rightX }
        )
        LauncherSubsystem.defaultCommand =
            LauncherSubsystem.run {
                LauncherSubsystem.LauncherMotor.set(-controller.leftTriggerAxis + controller.rightTriggerAxis)
            }
        controller.a().whileTrue(LauncherSubsystem.LAUNCH_FAST)
        controller.b().whileTrue(LauncherSubsystem.LAUNCH)
        controller.x().whileTrue(LauncherSubsystem.INTAKE)

        val park = if (robotType == RobotType.SWERVE) {
            controller.leftBumper().whileTrue(SwerveDriveSubsystem().PARK_COMMAND)
        } else {
            null
        }
    }
}