package org.firstinspires.ftc.teamcode

class MathUtils {
    companion object Helpers {
        fun preventInfinity(input: Double): Double {
            if (input == Double.POSITIVE_INFINITY) {
                return 0.0
            } else if (input == Double.NEGATIVE_INFINITY) {
                return 0.0;
            }

            return input;
        }
    }
}
