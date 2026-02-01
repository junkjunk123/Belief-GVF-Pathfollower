package belief;

import manifold.*;
import statistics.ProbabilityDensity;
import util.Pair;

public class Belief<
        Pose extends ManifoldPoint<Pose>,
        Twist extends TangentVector<Pose, Twist>,
        Workspace extends RiemannianManifold<Pose, Twist, BilinearForm<Twist>>>
        implements ManifoldPoint<Belief<Pose, Twist, Workspace>>,
        ProbabilityDensity<Pair<Pose, Twist>> {

    public final Pose muPose;
    public final Twist muTwist;
    public final BilinearForm<ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist>> sigma;
    public final BilinearForm<ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist>> sigmaInverse;
    public final ProductManifold<Pose, Twist, BilinearForm<Twist>, Pose, Twist, BilinearForm<Twist>, Workspace, Workspace> manifold;

    public final Workspace workspace;

    public Belief(
            Pose muPose,
            Twist muTwist,
            BilinearForm<ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist>> sigma,
            Workspace workspace
    ) {
        this.muPose = muPose;
        this.muTwist = muTwist;
        this.sigma = sigma;
        this.sigmaInverse = sigma.inverse();
        this.workspace = workspace;
        manifold = ProductManifold.of(workspace, workspace);
    }

    @Override
    public Belief<Pose, Twist, Workspace> copy() {
        return new Belief<>(muPose, muTwist, sigma, workspace);
    }

    /**
     * Intrinsic Gaussian density on (Pose, Twist)
     */
    @Override
    public double density(Pair<Pose, Twist> state) {
        Pose pose = state.one();
        Twist twist = state.two();

        /* Pose difference */
        Twist dPose = workspace.log(muPose, pose);

        /* Transport twist to mean pose */
        Twist twistAtMu =
                workspace.parallelTransport(pose, muPose, twist);

        /* Velocity difference */
        Twist dTwist = twistAtMu.add(muTwist.scale(-1));

        /* Concatenate into joint tangent */
        ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist> z = new ProductManifold.ProductTangentVector<>(dPose, dTwist);

        double quad = manifold.innerProduct(sigmaInverse.multiply(z), z);

        double norm =
                1.0 / Math.sqrt(
                        Math.pow(2 * Math.PI, 2 * workspace.dim()) *
                                sigma.determinant()
                );

        return norm * Math.exp(-0.5 * quad);
    }

    public double klDivergence(Belief<Pose, Twist, Workspace> other) {
        double trace = other.sigmaInverse.multiply(this.sigma).trace();
        double lnRatio = Math.log(other.sigma.determinant() / this.sigma.determinant());
        Twist dPose = workspace.log(muPose, other.muPose);
        Twist transportedTwist = workspace.parallelTransport(other.muPose, muPose, other.muTwist);
        Twist dTwist = transportedTwist.add(muTwist.scale(-1));
        ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist> dMu =
                new ProductManifold.ProductTangentVector<>(dPose, dTwist);
        double mahalanobis = manifold.quadraticForm(dMu, other.sigmaInverse, dMu);
        double k = 2 * workspace.dim();
        return 0.5 * (trace + lnRatio + mahalanobis - k);
    }
}
