package util;

public class Timer {
    private long startTime;
    private boolean started = false;

    public Timer() {
        reset();
    }

    public void start() {
        started = true;
    }

    public void reset() {
        startTime = System.nanoTime();
    }

    public long getNanoseconds() {
        if (!started) return 0;
        return System.nanoTime() - startTime;
    }

    public double getSeconds() {
        return getNanoseconds() * 1e-9;
    }

    public double lapSeconds() {
        if (!started) return 0;
        long now = System.nanoTime();
        double elapsed = (now - startTime) * 1e-9;
        startTime = now;
        return elapsed;
    }

    public boolean isStarted() {
        return started;
    }
}
