using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Solver
{
    class Release : Element
    {
        private double L { get; set; }

        public Release(Node startNode, Node endNode, int ID)
            : base(startNode, endNode, ID)
        {
            this.Type = "Release";
            this.L = 0;
            this.nodesAmount = 2;
            nodesList = new Node[] { startNode, endNode };
        }

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
            double[,] localStiffnessMatrix = new double[12, 12];

            localStiffnessMatrix = new double[,] { {   1,  0,  0, 0, 0, 0, -1,  0,  0, 0, 0, 0 },
                                                    {  0,  1,  0, 0, 0, 0,  0, -1,  0, 0, 0, 0 },
                                                    {  0,  0,  1, 0, 0, 0,  0,  0, -1, 0, 0, 0 },
                                                    {  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0 },
                                                    {  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0 },
                                                    {  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0 },
                                                    { -1,  0,  0, 0, 0, 0,  1,  0,  0, 0, 0, 0 },
                                                    {  0, -1,  0, 0, 0, 0,  0,  1,  0, 0, 0, 0 },
                                                    {  0,  0, -1, 0, 0, 0,  0,  0,  1, 0, 0, 0 },
                                                    {  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0 },
                                                    {  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0 },
                                                    {  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0 } };
          

            LocalStiffnessMatrix = DenseMatrix.OfArray(localStiffnessMatrix);
        }

        protected override void CalculateEquivalentNodalForcesVector()
        {
             EquivalentNodalForcesVector = new DenseVector(12);
        }

        public override void CalculateStress() { }
    }
}
