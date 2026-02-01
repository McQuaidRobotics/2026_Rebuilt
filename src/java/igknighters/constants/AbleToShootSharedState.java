package igknighters.constants;

public class AbleToShootSharedState {

    private AbleToShootSharedState() {}

    private static class SingletonHelper {
        private static final AbleToShootSharedState INSTANCE = new AbleToShootSharedState();
    }

    public static AbleToShootSharedState getInstance() {
        return SingletonHelper.INSTANCE;
    }

}
    