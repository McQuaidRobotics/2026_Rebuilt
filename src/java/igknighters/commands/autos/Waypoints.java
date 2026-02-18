package igknighters.commands.autos;

public enum Waypoints {
    STARTING_RIGHT,
    BUMP_LAND_RIGHT,
    BALLS_RIGHT,
    BALLS_MIDDLE,
    RIGHT,
    LEFT,
    CLIMB_LEFT;

    public String to(Waypoints wp) {
        return this.name() + "_TO_" + wp.name();
    }
}
