package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.Alliance

/**
 * Default auton: one straight path of about 20 in from the start pose to
 * [path1End], turning along the way (`linear`), then a hold until the
 * measured pose arrives (the path's parametric end is not arrival). RED
 * coordinates; BLUE is the season's 180° rotation. Lifecycle is [AutoBase]'s.
 */
@Autonomous(name = "Default Auto RED", group = "Match")
open class DefaultAutoRed : AutoBase() {

    /** The BLUE copy of this file overrides this and changes nothing else. */
    override val initialAlliance: Alliance get() = Alliance.RED
    override val routineName = "Default Auto"

    override val startPose: Pose get() = poses.of(56.0, 8.0, 90.0)
    private val path1End get() = poses.of(70.7572, 21.5438, 180.0)

    private fun path1(): Path = line(startPose, path1End).linear(startPose, path1End)

    override fun routine(): Command = sequential(
        follow("Drive path 1", path1()),
        drive.holdCommand(path1End, name = "Settle at path 1 end"),
    )
}

@Autonomous(name = "Default Auto BLUE", group = "Match")
class DefaultAutoBlue : DefaultAutoRed() {
    override val initialAlliance: Alliance get() = Alliance.BLUE
}
