package gvf;

import manifold.endomorphism.Endomorphism;
import manifold.ManifoldPoint;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import manifold.endomorphism.IdentityEndomorphism;
import mathematics.DifferentiableScalarFunction;
import util.Pair;

public abstract class GVFPath<Workspace extends RiemannianManifold<Pose, Vector, Matrix>,
        Pose extends ManifoldPoint<Pose>,
        Vector extends TangentVector<Pose, Vector>,
        Matrix extends Endomorphism<Vector>> implements IGVF<Pose, Vector, Matrix, Workspace> {
    private final DifferentiableScalarFunction<Pose> signedDistanceToPath;
    private final Workspace workspace;
    private final Pair<Double, Double> gvfCoefficients;
    private final Matrix modelCovariance;

    public GVFPath(DifferentiableScalarFunction<Pose> signedDistanceToPath, Workspace workspace,
                   Pair<Double, Double> gvfCoefficients, Matrix modelCovariance) {
        this.signedDistanceToPath = signedDistanceToPath;
        this.workspace = workspace;
        this.gvfCoefficients = gvfCoefficients;
        this.modelCovariance = modelCovariance;
    }

    protected abstract Vector biasField(Pose x);

    protected Endomorphism<Vector> tangentProjector(Pose x) {
        Vector gradient = workspace.gradient(x, signedDistanceToPath);
        Endomorphism<Vector> tensor = gradient.tensorProduct(gradient);
        Endomorphism<Vector> identity = new IdentityEndomorphism<>(workspace.dim());
        return tensor.scale(-1).add(identity);
    }

    public Vector evaluate(Pose pose) {
        Vector biasField = biasField(pose);
        Endomorphism<Vector> tangentProjector = tangentProjector(pose);
        double coeffOne = gvfCoefficients.one();
        double coeffTwo = gvfCoefficients.two();
        Vector tangent = tangentProjector.multiply(biasField).scale(coeffOne);
        double distance = signedDistanceToPath.evaluate(pose);
        Vector grad = workspace.gradient(pose, signedDistanceToPath);
        Vector normal = grad.scale(distance * coeffTwo);
        return tangent.add(normal);
    }

    @Override
    public Vector covariantDerivative() {
        //TODO: Fill out ts
        return null;
    }

    @Override
    public Matrix covariance() {
        return modelCovariance;
    }
}
