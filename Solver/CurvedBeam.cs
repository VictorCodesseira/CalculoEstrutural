using MathNet.Numerics.LinearAlgebra.Double;

namespace Solver
{
    class CurvedBeam : Element
    {
        Node centerNode;
        public CurvedBeam(Node startNode, Node endNode, Node centerNode, Material material, Section section, int ID)
            :base(startNode, endNode, material, section, ID)
        {
            this.Type = "Curved Beam";
            this.centerNode = centerNode;
        }

        public override void SetAddressTable()
        {
            int[] addressTable = new int[18];
            for (int i = 0; i < 6; i++)
            {
                addressTable[i] = startNode.ID * 6 + i;
                addressTable[i + 6] = centerNode.ID * 6 + i;
                addressTable[2*i + 6] = endNode.ID * 6 + i;
            }
            AddressTable = addressTable;
        }

        protected override void CalculateLocalStiffnessMatrix()
        {
            double[,] localStiffnessMatrix = new double[12, 12];
            LocalStiffnessMatrix = DenseMatrix.OfArray(localStiffnessMatrix);
        }

        protected override void CalculateEquivalentNodalForcesVector()
        {
            EquivalentNodalForcesVector = new DenseVector(12);
        }

        public override void CalculateStress()
        {
            return;
        }

    }
}
