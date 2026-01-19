package gvf;

import manifold.*;
import statistics.ProbabilityDensity;

import java.util.function.BiFunction;

public record Belief<Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>,
        Workspace extends RiemannianManifold<Pose, Twist>>(Pose mu,
                                                           RankTwoTensor<Twist> sigma) implements ManifoldPoint<Belief<Pose, Twist, Workspace>>,
        ProbabilityDensity<Pair<Pose, Twist>> {

    @Override
    public Belief<Pose, Twist, Workspace> copy() {
        return new Belief<>(mu, sigma);
    }

    @Override
    public double density(Pair<Pose, Twist> poseTwistPair) {
        return 0;
    }
}
