using MathNet.Numerics.LinearAlgebra.Double;

namespace Solver
{
    class Beam : Element
    {
        private DistributedLoad load;
        private double L { get; set; }

        public Beam(Node startNode, Node endNode, Material material, Section section, int ID)
            : base(startNode, endNode, material, section, ID)
        {
            this.Type = "Beam";
            this.L = startNode.Distance(endNode);
        }

        public void addLoad(DistributedLoad ld)
        {
            this.load = ld;
        }

        public double[] ElasticLineX, ElasticLineY, ElasticLineZ;
        public double[] NormalForce, ShearForceY, ShearForceZ;
        public double[] BendingMomentY, BendingMomentZ, Torsion;

        public override void SetAddressTable()
        {
            int[] addressTable = new int[12];
            for (int i = 0; i < 6; i++)
            {
                addressTable[i] = startNode.ID * 6 + i;
                addressTable[i + 6] = endNode.ID * 6 + i;
            }
            AddressTable = addressTable;
        }

        protected override void CalculateLocalStiffnessMatrix()
        {
            double[,] localStiffnessMatrix = new double[12,12];
            double X = E * A / L;
            if (!(startNode.hinge || endNode.hinge)) // Regular Beam
            {
                double Y1 = 12 * E * Iz / (L * L * L);
                double Z1 = 12 * E * Iy / (L * L * L);
                double Y2 = 6 * E * Iz / (L * L);
                double Z2 = 6 * E * Iy / (L * L);
                double S = G * J / L;
                double Y3 = 4 * E * Iz / L;
                double Z3 = 4 * E * Iy / L;
                double Y4 = Y3 / 2;
                double Z4 = Z3 / 2;

                localStiffnessMatrix = new double[,] { {  X,   0,   0,  0,   0,   0, -X,   0,   0,  0,   0,   0 },
                                                       {  0,  Y1,   0,  0,   0,  Y2,  0, -Y1,   0,  0,   0,  Y2 },
                                                       {  0,   0,  Z1,  0, -Z2,   0,  0,   0, -Z1,  0, -Z2,   0 },
                                                       {  0,   0,   0,  S,   0,   0,  0,   0,   0, -S,   0,   0 },
                                                       {  0,   0, -Z2,  0,  Z3,   0,  0,   0,  Z2,  0,  Z4,   0 },
                                                       {  0,  Y2,   0,  0,   0,  Y3,  0, -Y2,   0,  0,   0,  Y4 },
                                                       { -X,   0,   0,  0,   0,   0,  X,   0,   0,  0,   0,   0 },
                                                       {  0, -Y1,   0,  0,   0, -Y2,  0,  Y1,   0,  0,   0, -Y2 },
                                                       {  0,   0, -Z1,  0,  Z2,   0,  0,   0,  Z1,  0, -Z2,   0 },
                                                       {  0,   0,   0, -S,   0,   0,  0,   0,   0,  S,   0,   0 },
                                                       {  0,   0, -Z2,  0,  Z4,   0,  0,   0, -Z2,  0,  Z3,   0 },
                                                       {  0,  Y2,   0,  0,   0,  Y4,  0, -Y2,   0,  0,   0,  Y3 } };
            }
            else if (startNode.hinge && endNode.hinge) // Truss
            {
                localStiffnessMatrix[0, 0] = localStiffnessMatrix[6, 6] = X;
                localStiffnessMatrix[0, 6] = localStiffnessMatrix[6, 0] = -X;
            }
            else
            {
                double Y1 = 3 * E * Iz / (L * L * L);
                double Z1 = 3 * E * Iy / (L * L * L);
                double Y2 = 3 * E * Iz / (L * L);
                double Z2 = 3 * E * Iy / (L * L);
                double Y3 = 3 * E * Iz / L;
                double Z3 = 3 * E * Iy / L;
                if (startNode.hinge) // Hinge at start
                {
                    localStiffnessMatrix = new double[,] { {  X,   0,   0, 0, 0, 0, -X,   0,   0, 0,   0,   0 },
                                                           {  0,  Y1,   0, 0, 0, 0,  0, -Y1,   0, 0,   0,  Y2 },
                                                           {  0,   0,  Z1, 0, 0, 0,  0,   0, -Z1, 0, -Z2,   0 },
                                                           {  0,   0,   0, 0, 0, 0,  0,   0,   0, 0,   0,   0 },
                                                           {  0,   0,   0, 0, 0, 0,  0,   0,   0, 0,   0,   0 },
                                                           {  0,   0,   0, 0, 0, 0,  0,   0,   0, 0,   0,   0 },
                                                           { -X,   0,   0, 0, 0, 0,  X,   0,   0, 0,   0,   0 },
                                                           {  0, -Y1,   0, 0, 0, 0,  0,  Y1,   0, 0,   0, -Y2 },
                                                           {  0,   0, -Z1, 0, 0, 0,  0,   0,  Z1, 0, -Z2,   0 },
                                                           {  0,   0,   0, 0, 0, 0,  0,   0,   0, 0,   0,   0 },
                                                           {  0,   0, -Z2, 0, 0, 0,  0,   0, -Z2, 0,  Z3,   0 },
                                                           {  0,  Y2,   0, 0, 0, 0,  0, -Y2,   0, 0,   0,  Y3 } };
                }
                else // Hinge at end
                {
                    localStiffnessMatrix = new double[,] { {  X,   0,   0, 0,   0,   0, -X,   0,   0, 0, 0, 0 },
                                                           {  0,  Y1,   0, 0,   0,  Y2,  0, -Y1,   0, 0, 0, 0 },
                                                           {  0,   0,  Z1, 0, -Z2,   0,  0,   0, -Z1, 0, 0, 0 },
                                                           {  0,   0,   0, 0,   0,   0,  0,   0,   0, 0, 0, 0 },
                                                           {  0,   0, -Z2, 0,  Z3,   0,  0,   0, -Z2, 0, 0, 0 },
                                                           {  0,  Y2,   0, 0,   0,  Y3,  0, -Y2,   0, 0, 0, 0 },
                                                           { -X,   0,   0, 0,   0,   0,  X,   0,   0, 0, 0, 0 },
                                                           {  0, -Y1,   0, 0,   0, -Y2,  0,  Y1,   0, 0, 0, 0 },
                                                           {  0,   0, -Z1, 0, -Z2,   0,  0,   0,  Z1, 0, 0, 0 },
                                                           {  0,   0,   0, 0,   0,   0,  0,   0,   0, 0, 0, 0 },
                                                           {  0,   0,   0, 0,   0,   0,  0,   0,   0, 0, 0, 0 },
                                                           {  0,   0,   0, 0,   0,   0,  0,   0,   0, 0, 0, 0 } };
                }
            }
            LocalStiffnessMatrix = DenseMatrix.OfArray(localStiffnessMatrix);
        }

        protected override void CalculateEquivalentNodalForcesVector()
        {
            if (load == null)
            {
                EquivalentNodalForcesVector = new DenseVector(12);
                return;
            }
            double[] sv = load.startValue;
            double[] ev = load.endValue;
            double[] loads = { ((1.0 / 3.0) * sv[0] + (1.0 / 6.0) * ev[0])*L,
                               ((7.0 / 20.0) * sv[1] + (3.0 / 20.0) * ev[1])*L,
                               ((7.0 / 20.0) * sv[2] + (3.0 / 20.0) * ev[2])*L,
                               0,
                               ((1.0 / 20.0) * sv[2] + (1.0 / 30.0) * ev[2])*L*L,
                               ((1.0 / 20.0) * sv[1] + (1.0 / 30.0) * ev[1])*L*L,
                               ((1.0 / 6.0) * sv[0] + (1.0 / 3.0) * ev[0])*L,
                               ((3.0 / 20.0) * sv[1] + (7.0 / 20.0) * ev[1])*L,
                               ((3.0 / 20.0) * sv[2] + (7.0 / 20.0) * ev[2])*L,
                               0,
                               (-(1.0 / 30.0) * sv[2] - (1.0 / 20.0) * ev[2])*L*L,
                               (-(1.0 / 30.0) * sv[1] - (1.0 / 20.0) * ev[1])*L*L };
            EquivalentNodalForcesVector = DenseVector.OfArray(loads);
        }

        private void CalculateElasticLines()
        {
            DenseVector delta = NodalDisplacementsVector;
            ElasticLineX = new double[] { delta[0],
                                          1 + (delta[6] - delta[0])/L };

            double w0, wL, phi0, phiL, a0, b0, I;
            a0 = 0;
            b0 = 0;
            double L2 = L * L, L3 = L2 * L, L4 = L3 * L, L5 = L4 * L;
            w0 = delta[1];
            wL = delta[7];
            phi0 = delta[5];
            phiL = delta[11];
            I = Iz;
            if (load != null)
            {
                a0 = (load.endValue[1] - load.startValue[1]) / L;
                b0 = load.startValue[1];
            }
            ElasticLineY = new double[] { w0,
                                          phi0,
                                          (2 * L5 * a0 + 5 * L4 * b0 - 360 * E * I * w0 + 360 * E * I * wL - 240 * E * I * L * phi0 - 120 * E * I * L * phiL) / (120 * E * I * L2),
                                          -(3 * L5 * a0 + 10 * L4 * b0 - 240 * E * I * w0 + 240 * E * I * wL - 120 * E * I * L * phi0 - 120 * E * I * L * phiL) / (120 * E * I * L3),
                                          b0 / (24 * E * I),
                                          a0 / (120 * E * I) };

            w0 = delta[2];
            wL = delta[8];
            phi0 = delta[4];
            phiL = delta[10];
            I = Iy;
            if (load != null)
            {
                a0 = (load.endValue[2] - load.startValue[2]) / L;
                b0 = load.startValue[2];
            }
            ElasticLineZ = new double[] { w0,
                                          phi0,
                                          (2 * L5 * a0 + 5 * L4 * b0 - 360 * E * I * w0 + 360 * E * I * wL - 240 * E * I * L * phi0 - 120 * E * I * L * phiL) / (120 * E * I * L2),
                                          -(3 * L5 * a0 + 10 * L4 * b0 - 240 * E * I * w0 + 240 * E * I * wL - 120 * E * I * L * phi0 - 120 * E * I * L * phiL) / (120 * E * I * L3),
                                          b0 / (24 * E * I),
                                          a0 / (120 * E * I) };
        }


        public override void CalculateStress()
        {
            if (this.ElasticLineX == null)
            {
                CalculateElasticLines();
            }
            DenseVector startLoad = startNode.load.FVector;
            NormalForce = new double[] { (ElasticLineX[1] - 1)*E*A };
            BendingMomentY = new double[] { 2 * E * Iy * ElasticLineZ[2], 6 * E * Iy * ElasticLineZ[3], 12 * E * Iy * ElasticLineZ[4], 20 * E * Iy * ElasticLineZ[5] };
            BendingMomentZ = new double[] { 2 * E * Iz * ElasticLineY[2], 6 * E * Iz * ElasticLineY[3], 12 * E * Iz * ElasticLineY[4], 20 * E * Iz * ElasticLineY[5] };
            ShearForceY = new double[] { BendingMomentZ[1], 2 * BendingMomentZ[2], 3 * BendingMomentZ[3] };
            ShearForceZ = new double[] { BendingMomentY[1], 2 * BendingMomentY[2], 3 * BendingMomentY[3] };

            Torsion = new double[] { NodalDisplacementsVector[9]-NodalDisplacementsVector[3]*G*J/L };
        }

    }
}
