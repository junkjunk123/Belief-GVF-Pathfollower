package solver;

import manifold.*;
import statistics.ProbabilityDensity;

public class Belief<Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>, MatrixType extends Matrix<Twist>,
        Workspace extends RiemannianManifold<Pose, Twist, MatrixType>>
        implements ManifoldPoint<Belief<Pose, Twist, MatrixType, Workspace>>,
        ProbabilityDensity<Pair<Pose, Twist>> {
    public final ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
            RiemannianManifold<Pose, Twist, MatrixType>,
            RiemannianManifold<Pose, Twist, MatrixType>>.ProductPoint mu;
    public final Matrix<ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
        RiemannianManifold<Pose, Twist, MatrixType>,
        RiemannianManifold<Pose, Twist, MatrixType>>.ProductTangentVector> sigma;
    public final Matrix<ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
            RiemannianManifold<Pose, Twist, MatrixType>,
            RiemannianManifold<Pose, Twist, MatrixType>>.ProductTangentVector> sigmaInverse;
    public final ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
        RiemannianManifold<Pose, Twist, MatrixType>,
        RiemannianManifold<Pose, Twist, MatrixType>> manifold;
    public final RiemannianManifold<Pose,Twist, MatrixType> workspace;
    public final GuidingVectorField<Pose, Twist, MatrixType, Workspace> referenceVectorField;

    public Belief(
            ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
                    RiemannianManifold<Pose, Twist, MatrixType>,
                    RiemannianManifold<Pose, Twist, MatrixType>>.ProductPoint mu,
            Matrix<ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
                    RiemannianManifold<Pose, Twist, MatrixType>,
                    RiemannianManifold<Pose, Twist, MatrixType>>.ProductTangentVector> sigma,
            RiemannianManifold<Pose, Twist, MatrixType> workspace,
            GuidingVectorField<Pose, Twist, MatrixType, Workspace> referenceVectorField
    ) {
        this.mu = mu;
        this.sigma = sigma;
        sigmaInverse = sigma.inverse();
        this.workspace = workspace;
        manifold = ProductManifold.of(workspace, workspace);
        this.referenceVectorField = referenceVectorField;
    }

    @Override
    public Belief<Pose, Twist, MatrixType, Workspace> copy() {
        return new Belief<>(mu, sigma, workspace, referenceVectorField);
    }

    @Override
    public double density(Pair<Pose, Twist> poseTwistPair) {
        double scalar = 1.0 / Math.sqrt(Math.pow(2 * Math.PI, manifold.dim()) * sigma.determinant());
        Twist log = workspace.log(mu.points.one(), poseTwistPair.one());
        Twist parallelTransport = workspace.parallelTransport(poseTwistPair.one(), mu.points.one(), poseTwistPair.two());
        Twist diff = parallelTransport.add(referenceVectorField.apply(mu.points.one()).scale(-1));
        ProductManifold<Pose, Twist, MatrixType, Pose, Twist, MatrixType,
                RiemannianManifold<Pose, Twist, MatrixType>,
                RiemannianManifold<Pose, Twist, MatrixType>>.ProductTangentVector concat = manifold.concat(log, diff);
        double exp = -0.5 * manifold.innerProduct(concat, sigmaInverse.multiply(concat));
        return scalar * Math.exp(exp);
    }
}
