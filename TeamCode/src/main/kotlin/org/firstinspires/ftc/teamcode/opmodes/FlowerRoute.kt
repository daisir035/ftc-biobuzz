package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.PoseFactory
import com.pedropathing.api.Paths.curve
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path

/**
 * The FLOWER cycle route, shared by "Flower Cycle" and "Just Path" so a pose
 * is edited in one place. RED coordinates (inches, degrees) through [poses],
 * which also maps them for BLUE. Every path turns along its length.
 *
 * start → first FLOWER → far shot → second FLOWER → home shot.
 * Open so tests can substitute a small synthetic course.
 */
open class FlowerRoute(protected val poses: PoseFactory) {
    open val start: Pose get() = poses.of(57.3401, 7.4974, -90.0)
    open val firstFlower: Pose get() = poses.of(23.3387, 42.5526, 180.0)
    open val farShot: Pose get() = poses.of(66.0339, 115.8782, 90.0)
    open val secondFlower: Pose get() = poses.of(55.782, 112.4703, 90.0)
    open val homeShot: Pose get() = poses.of(65.18, 116.5975, 90.0)

    open fun toFirstFlower(): Path = curve(
        start,
        poses.of(37.805, 49.4391, 0.0),
        firstFlower,
    ).linear(start, firstFlower)

    open fun toFarShot(): Path = curve(
        firstFlower,
        poses.of(44.1777, 64.5854, 0.0),
        poses.of(0.9587, 90.9097, 0.0),
        poses.of(31.6563, 124.5746, 0.0),
        poses.of(69.5067, 96.8234, 0.0),
        farShot,
    ).linear(firstFlower, farShot)

    open fun toSecondFlower(): Path = curve(
        farShot,
        poses.of(62.4671, 100.26, 0.0),
        poses.of(45.2203, 99.5701, 0.0),
        secondFlower,
    ).linear(farShot, secondFlower)

    open fun toHomeShot(): Path = curve(
        secondFlower,
        poses.of(55.5484, 86.4742, 0.0),
        homeShot,
    ).linear(secondFlower, homeShot)
}
