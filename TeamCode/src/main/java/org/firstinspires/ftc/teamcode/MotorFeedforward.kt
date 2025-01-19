package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import kotlin.math.withSign

data class MotorFeedforward(
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kA: Double,
    @JvmField
    val packet: TelemetryPacket
) {
    /**
     * @usesMathJax
     *
     * Computes the (normalized) voltage \(k_s \cdot \operatorname{sign}(k_v \cdot v + k_a \cdot a) + k_v \cdot v + k_a \cdot a\).
     *
     * @param[vel] \(v\)
     * @param[accel] \(a\)
     */
    fun compute(vel: Double, accel: Double): Double {
        val ks = kS
        val kv = kV * vel
        val ka = kA * accel

        packet.put("ks", ks)
        packet.put("kv", kv)
        packet.put("ka", ka)

        return ks + kv + ka
    }

    fun compute(vel: DualNum<Time>) = compute(vel[0], vel[1])
}
