package childposepredictor.Resources;

import java.util.HashMap;
import java.util.Map;

public class BallTracker {
    private final Map<Integer, Predictor> ballPredictors = new HashMap<>();

    public BallTracker() {
        for (int i = 1; i <= 4; i++) {
            ballPredictors.put(i, new Predictor());
        }
    }

    public void updateBall(int id, Translation3d newPos, double timestamp) {
        if (ballPredictors.containsKey(id)) {
            ballPredictors.get(id).addObservation(newPos, timestamp);
        }
    }

    public Translation3d getPredictedBallPos(int id, double lookahead) {
        return ballPredictors.get(id).predictFutureLocation(lookahead);
    }
}