package igknighters.util;

public class LerpTable {
    public static class LerpTableEntry {
        public double x;
        public double y;

        public LerpTableEntry(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double lerp(LerpTableEntry other, double x) {
            return this.y + (other.y - this.y) * ((x - this.x) / (other.x - this.x));
        }
    }

    private LerpTableEntry[] table;

    public LerpTable(LerpTableEntry... table) {
        this.table = table;
    }

    public double lerp(double x) {
        if (x < table[0].x) {
            return table[0].y;
        } else if (x > table[table.length - 1].x) {
            return table[table.length - 1].y;
        }

        for (int i = 0; i < table.length - 1; i++) {
            if (x >= table[i].x && x <= table[i + 1].x) {
                return table[i].lerp(table[i + 1], x);
            }
        }

        return 0;
    }

    public double lerpKeepSign(double x) {
        return lerp(Math.abs(x)) * Math.signum(x);
    }

    public double inverseLerp(double y) {
        // 1. Handle Out-of-Bounds (Clamping)
        // Note: This assumes Y values are increasing.
        // If your table is descending, swap the logic.
        if (y <= table[0].y) {
            return table[0].x;
        } else if (y >= table[table.length - 1].y) {
            return table[table.length - 1].x;
        }

        // 2. Search for the segment containing 'y'
        for (int i = 0; i < table.length - 1; i++) {
            LerpTableEntry p0 = table[i];
            LerpTableEntry p1 = table[i + 1];

            // Check if y falls between these two points
            // (Math.min/max handles both increasing and decreasing segments)
            if (y >= Math.min(p0.y, p1.y) && y <= Math.max(p0.y, p1.y)) {
                // Prevent division by zero if two points have the same Y
                if (Math.abs(p1.y - p0.y) < 1e-9) {
                    return p0.x;
                }

                // The Inverse Lerp Formula:
                return p0.x + (y - p0.y) * ((p1.x - p0.x) / (p1.y - p0.y));
            }
        }

        return 0;
    }
}
