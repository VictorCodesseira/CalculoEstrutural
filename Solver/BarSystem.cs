using MathNet.Numerics.LinearAlgebra.Double;

namespace Solver
{
    class BarSystem
    {
        public Node[] Nodes;
        public Element[] Elements;

        public BarSystem(Node[] NodesList, Element[] ElementsList)
        {
            this.Nodes = NodesList;
            this.Elements = ElementsList;
        }

        private DenseMatrix SystemStiffnessMatrix;
        private DenseVector NodalForcesVector;
        private DenseVector NodalDisplacementsVector;
        private DenseVector ReactionsVector;

        private void AssembleSystemStiffnessMatrix()
        {
            int Nodes_n = this.Nodes.GetLength(0);
            DenseMatrix Aux = new DenseMatrix(Nodes_n * 6);
            foreach (Element element in Elements)
            {
                DenseMatrix ElementStiffness = element.GlobalStiffnessMatrix;
                int[] FreedomTable = element.AddressTable;
                int size = FreedomTable.Length;
                for (int i = 0; i < size; i++)
                {
                    int line = FreedomTable[i];
                    for (int j = 0; j < size; j++)
                    {
                        int column = FreedomTable[j];
                        Aux[line, column] += ElementStiffness[i, j];
                    }
                }
            }

            SystemStiffnessMatrix = Aux;
        }

        private void AssembleNodalForcesVector()
        {
            int Nodes_n = this.Nodes.GetLength(0);
            DenseVector Aux = new DenseVector(Nodes_n * 6);
            foreach (Node node in Nodes)
            {
                node.load.FVector.CopySubVectorTo(Aux, 0, node.ID*6, 6);
            }
            foreach (Element element in Elements)
            {
                DenseVector ElementNodalForces = element.GlobalEquivalentNodalForcesVector;
                int[] FreedomTable = element.AddressTable;
                for (int i = 0; i < FreedomTable.Length; i++)
                {
                    int line = FreedomTable[i];
                    Aux[line] += ElementNodalForces[i];
                }
            }

            NodalForcesVector = Aux;
        }

        private void CalculateNodalDisplacements()
        {
            DenseMatrix RestrictedStiffnessMatrix = DenseMatrix.OfMatrix(SystemStiffnessMatrix);
            DenseVector RestrictedForcesVector = DenseVector.OfVector(NodalForcesVector);

            int Nodes_n = this.Nodes.GetLength(0);
            for (int i = 0; i < Nodes_n; i++){
                Node node = Nodes[i];
                for (int r = 0; r < 6; r++)
                {
                    if (node.link.restrictions[r] == 1 || node.ForcedDisplacements[r] != 0)
                    {
                        int DOF_n = 6 * i + r;
                        RestrictedForcesVector[DOF_n] = node.ForcedDisplacements[r];
                        for (int j = 0; j < Nodes_n * 6; j++)
                        {
                            if (j != DOF_n)
                            {
                                RestrictedForcesVector[j] -= RestrictedStiffnessMatrix[j, DOF_n]*node.ForcedDisplacements[r];
                            }
                            RestrictedStiffnessMatrix[DOF_n, j] = 0;
                            RestrictedStiffnessMatrix[j, DOF_n] = 0;
                        }
                        RestrictedStiffnessMatrix[DOF_n, DOF_n] = 1;

                    }
                }
            }

            NodalDisplacementsVector = DenseVector.OfVector(RestrictedStiffnessMatrix.Solve(RestrictedForcesVector));
        }

        private void CalculateReactions()
        {
            System.Console.Write(SystemStiffnessMatrix.ToMatrixString(12,12));
            System.Console.Write(NodalDisplacementsVector.ToVectorString(12,12));
            ReactionsVector = SystemStiffnessMatrix * NodalDisplacementsVector;
        }

        public void Solve()
        {
            foreach (Element element in Elements)
            {
                element.CalculateMatrices();
            }

            AssembleSystemStiffnessMatrix();
            AssembleNodalForcesVector();
            CalculateNodalDisplacements();
            CalculateReactions();

            foreach (Node node in Nodes)
            {
                int nodeID = node.ID;
                NodalDisplacementsVector.CopySubVectorTo(node.NodalDisplacements, 6 * nodeID, 0, 6);
                ReactionsVector.CopySubVectorTo(node.EndForces, 6 * nodeID, 0, 6);
            }
            foreach (Element element in Elements)
            {
                element.CalculateStress();
            }
        }
    }
}
