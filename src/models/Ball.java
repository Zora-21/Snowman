package models;

public class Ball {
    private final String id;
    private final int intId; // 0..N-1, assigned at construction
    private int size;
    private int locId; // Graph Node ID

    public Ball(String id, int intId, int size, int locId) {
        this.id = id;
        this.intId = intId;
        this.size = size;
        this.locId = locId;
    }

    public String getId() {
        return id;
    }

    public int getIntId() {
        return intId;
    }

    public int getSize() {
        return size;
    }

    public int getLocId() {
        return locId;
    }

    public void setSize(int size) {
        this.size = size;
    }

    public void setLocId(int locId) {
        this.locId = locId;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        return intId == ((Ball) o).intId;
    }

    @Override
    public int hashCode() {
        return intId;
    }

    @Override
    public String toString() {
        return "Ball{id='" + id + "'(" + intId + "), size=" + size + ", locId=" + locId + "}";
    }
}