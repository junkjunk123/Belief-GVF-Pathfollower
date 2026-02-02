package belief;

import manifold.*;
import manifold.endomorphism.Endomorphism;
import mathematics.DifferentiableScalarFunction;

import java.util.function.BiFunction;

public class BeliefManifold<Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>,
        Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>>
        implements RiemannianManifold<Belief<Pose, Twist, Workspace>, BeliefVector<Pose, Twist, Workspace>,
        Endomorphism<BeliefVector<Pose, Twist, Workspace>>> {

    private final Workspace workspace;
    private final ProductManifold<Pose, Twist, Endomorphism<Twist>, Pose, Twist, Endomorphism<Twist>, Workspace, Workspace> manifold;

    public BeliefManifold(Workspace workspace) {
        this.workspace = workspace;
        this.manifold = ProductManifold.of(workspace, workspace);
    }

    @Override
    public BiFunction<BeliefVector<Pose, Twist, Workspace>, BeliefVector<Pose, Twist, Workspace>, Double> metric(Belief<Pose, Twist, Workspace> point) {
        return (uVec, vVec) -> {
            double meanTerm = manifold.quadraticForm(uVec.deltaMu(), point.sigmaInverse, vVec.deltaMu());
            double covTerm = 0.5 * point.sigmaInverse
                    .multiply(uVec.deltaSigma())
                    .multiply(point.sigmaInverse)
                    .multiply(vVec.deltaSigma())
                    .trace();
            return meanTerm + covTerm;
        };
    }

    @Override
    public int dim() {
        //parameterized by mu, sigma
        return 3 * workspace.dim() + 2 * workspace.dim() * workspace.dim();
    }

    @Override
    public Belief<Pose, Twist, Workspace> exp(Belief<Pose, Twist, Workspace> point, BeliefVector<Pose, Twist, Workspace> vector) {
        Pose newMuPose = workspace.exp(point.muPose, vector.deltaMu().vectors.one());
        Twist newMuTwist = workspace.parallelTransport(point.muPose, newMuPose, vector.deltaMu().vectors.two())
                .add(point.muTwist);

        var newSigma = point.sigma.add(vector.deltaSigma());

        return new Belief<>(newMuPose, newMuTwist, newSigma, workspace);
    }

    @Override
    public BeliefVector<Pose, Twist, Workspace> log(Belief<Pose, Twist, Workspace> start, Belief<Pose, Twist, Workspace> end) {
        Twist dPose = workspace.log(start.muPose, end.muPose);
        Twist dTwist = end.muTwist.add(start.muTwist.scale(-1));

        var deltaMu = new ProductManifold.ProductTangentVector<>(dPose, dTwist);
        var deltaSigma = end.sigma.add(start.sigma.scale(-1));

        return new BeliefVector<>(start, deltaMu, deltaSigma, workspace);
    }

    @Override
    public BeliefVector<Pose, Twist, Workspace> gradient(Belief<Pose, Twist, Workspace> point, DifferentiableScalarFunction<Belief<Pose, Twist, Workspace>> function) {
        throw new UnsupportedOperationException("Gradient not implemented for BeliefManifold.");
    }

    @Override
    public BeliefVector<Pose, Twist, Workspace> parallelTransport(Belief<Pose, Twist, Workspace> start, Belief<Pose, Twist, Workspace> end, BeliefVector<Pose, Twist, Workspace> vector) {
        throw new UnsupportedOperationException("Parallel transport not implemented for BeliefManifold.");
    }
}
