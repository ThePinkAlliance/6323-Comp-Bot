package org.firstinspires.ftc.teamcode

import kotlin.math.max
import kotlin.math.min

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

        /**
         * Returns value clamped between low and high boundaries.
         *
         * @param value Value to clamp.
         * @param low The lower boundary to which to clamp value.
         * @param high The higher boundary to which to clamp value.
         * @return The clamped value.
         */
        fun clamp(value: Double, low: Double, high: Double): Double {
            return max(low, min(value, high))
        }
    }
}
