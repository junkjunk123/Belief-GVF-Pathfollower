package statistics;

@FunctionalInterface
public interface ProbabilityDensity<U> {
    double density(U u);
}
