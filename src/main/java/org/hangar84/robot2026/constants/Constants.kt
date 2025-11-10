package org.hangar84.robot2026.constants

object Constants {

    object Mecanum {
        const val FRONT_LEFT_ID = 2
        const val FRONT_RIGHT_ID = 1
        const val REAR_LEFT_ID = 3
        const val REAR_RIGHT_ID = 4
    }

    object Swerve {
        const val FRONT_LEFT_DRIVING_ID = 4
        const val FRONT_LEFT_TURNING_ID = 3

        const val FRONT_RIGHT_DRIVING_ID = 1
        const val FRONT_RIGHT_TURNING_ID = 2

        const val REAR_LEFT_DRIVING_ID = 8
        const val REAR_LEFT_TURNING_ID = 7

        const val REAR_RIGHT_DRIVING_ID = 5
        const val REAR_RIGHT_TURNING_ID = 6
    }
    object LauncherSubsystem {
        const val LauncherMotor = 9
        const val FollowerLauncherMotor = 10
    }
}