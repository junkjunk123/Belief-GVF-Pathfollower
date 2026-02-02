package belief;

import manifold.*;
import manifold.endomorphism.Endomorphism;

import java.util.function.BiFunction;

public record BeliefVector<Pose extends ManifoldPoint<Pose>,
        Twist extends TangentVector<Pose, Twist>,
        Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>>
        (Belief<Pose, Twist, Workspace> belief,
         ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist> deltaMu,
         ProductManifold.ProductEndomorphism<Pose, Twist, Pose, Twist> deltaSigma,
         Workspace workspace,
         ProductManifold<Pose, Twist, Endomorphism<Twist>, Pose, Twist, Endomorphism<Twist>, Workspace, Workspace> manifold)
        implements TangentVector<Belief<Pose, Twist, Workspace>,
        BeliefVector<Pose, Twist, Workspace>>, BiFunction<Pose, Twist, Double> {

    public BeliefVector(
            Belief<Pose, Twist, Workspace> belief,
            ProductManifold.ProductTangentVector<Pose, Twist, Pose, Twist> deltaMu,
            ProductManifold.ProductEndomorphism<Pose, Twist, Pose, Twist> deltaSigma,
            Workspace workspace
    ) {
        this(belief, deltaMu, deltaSigma, workspace, ProductManifold.of(workspace, workspace));
    }

    @Override
    public Belief<Pose, Twist, Workspace> basePoint() {
        return belief;
    }

    @Override
    public BeliefVector<Pose, Twist, Workspace> add(BeliefVector<Pose, Twist, Workspace> other) {
        return new BeliefVector<>(
                belief,
                deltaMu.add(other.deltaMu),
                deltaSigma.add(other.deltaSigma),
                workspace
        );
    }

    @Override
    public BeliefVector<Pose, Twist, Workspace> scale(double alpha) {
        return new BeliefVector<>(
                belief,
                deltaMu.scale(alpha),
                deltaSigma.scale(alpha),
                workspace
        );
    }

    @Override
    public Endomorphism<BeliefVector<Pose, Twist, Workspace>> tensorProduct(BeliefVector<Pose, Twist, Workspace> other) {
        throw new UnsupportedOperationException("Tensor product not implemented for BeliefVector.");
    }

    @Override
    public Double apply(Pose pose, Twist twist) {
        /* Build joint tangent z = (dq, dv) */
        Twist dPose = workspace.log(belief.muPose, pose);
        Twist twistAtMu = workspace.parallelTransport(pose, belief.muPose, twist);
        Twist dTwist = twistAtMu.add(belief.muTwist.scale(-1));
        var z = new ProductManifold.ProductTangentVector<>(dPose, dTwist);

        /* Σ⁻¹ z */
        var SigmaInv_z = belief.sigmaInverse.multiply(z);

        /* First term: <Σ⁻¹ z, δμ> */
        var product = ProductManifold.of(workspace, workspace);
        double term1 = product.innerProduct(SigmaInv_z, deltaMu);

        /* Second term: <Σ⁻¹ (z ⊗ z) Σ⁻¹ − Σ⁻¹, δΣ> */
        var zzT = z.tensorProduct(z);

        var quadratic = belief.sigmaInverse
                        .multiply(zzT)
                        .multiply(belief.sigmaInverse)
                        .subtract(belief.sigmaInverse);

        double term2 = 0.5 * quadratic.innerProduct(deltaSigma);

        return term1 - term2;
    }
}
