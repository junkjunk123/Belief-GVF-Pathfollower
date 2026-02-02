package mathematics;

@FunctionalInterface
public interface ProbabilityDensity<U> {
    double density(U u);
}
