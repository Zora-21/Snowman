package models;

import java.util.List;

public class Grouping {
    private final List<Ball> balls;
    private final int[] ballIntIds;

    public Grouping(Ball b0, Ball b1, Ball b2) {
        this.balls = java.util.Arrays.asList(b0, b1, b2);
        this.ballIntIds = new int[] { b0.getIntId(), b1.getIntId(), b2.getIntId() };
    }

    public List<Ball> getBalls() {
        return balls;
    }

    public int[] getBallIntIds() {
        return ballIntIds;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("Grouping{");
        for (int i = 0; i < balls.size(); i++) {
            if (i > 0)
                sb.append(", ");
            sb.append(balls.get(i));
        }
        sb.append("}");
        return sb.toString();
    }
}