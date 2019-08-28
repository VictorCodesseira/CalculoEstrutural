using MathNet.Numerics.LinearAlgebra.Double;
using Newtonsoft.Json;
using System;

namespace Solver
{
    abstract class Element
    {
        [JsonProperty(Order = -3)]
        public string Type { get; protected set; }
        [JsonProperty(Order = -2)]
        public int ID;
        
        public Node startNode, endNode;
        protected Node[] nodesList;
        protected int nodesAmount;

        [JsonIgnore]
        public int[] AddressTable { get; protected set; }

        [JsonIgnore]
        public DenseMatrix RotationMatrix { get; protected set; }

        [JsonIgnore]
        public DenseMatrix LocalStiffnessMatrix { get; protected set; }

        [JsonIgnore]
        public DenseVector EquivalentNodalForcesVector { get; protected set; }

        public Element(Node startNode, Node endNode, int ID)
        {
            this.startNode = startNode;
            this.endNode = endNode;
            this.ID = ID;
        }

        public void CalculateMatrices()
        {
            SetAddressTable();
            CalculateRotationMatrix();
            CalculateLocalStiffnessMatrix();
            CalculateEquivalentNodalForcesVector();
        }

        [JsonIgnore]
        public DenseMatrix GlobalStiffnessMatrix
        {
            get
            {
                return RotationMatrix * LocalStiffnessMatrix * DenseMatrix.OfMatrix(RotationMatrix.Transpose());
            }
        }

        [JsonIgnore]
        public DenseVector NodalDisplacementsVector
        {
            get
            {
                DenseVector GlobalDisplacements = new DenseVector(nodesAmount*6);

                for (int i = 0; i < nodesAmount; i++)
                {
                    nodesList[i].NodalDisplacements.CopySubVectorTo(GlobalDisplacements, 0, 6 * i, 6);
                }

                return DenseVector.OfVector(RotationMatrix.Inverse() * GlobalDisplacements);
            }
        }

        [JsonIgnore]
        public DenseVector GlobalEquivalentNodalForcesVector
        {
            get
            {
                return RotationMatrix * EquivalentNodalForcesVector;
            }
        }

        abstract public void SetAddressTable();
        abstract protected void CalculateLocalStiffnessMatrix();
        abstract protected void CalculateEquivalentNodalForcesVector();
        abstract public void CalculateStress();

        private void CalculateRotationMatrix()
        {
            Vector3D startPos = startNode.Position, endPos = endNode.Position;
            double dx, dy, dz;
            double nx, ny, nz;
            double theta, alpha1, alpha2;

            dx = endPos.x - startPos.x;
            dy = endPos.y - startPos.y;
            dz = endPos.z - startPos.z;

            alpha1 = Math.Atan2(dy, dx);
            alpha2 = Math.Atan2(dz, Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2)));

            DenseVector n = (new DenseVector(new double[] { 0, -1.0 * Math.Sin(alpha2), Math.Sin(alpha1) * Math.Cos(alpha2) }));
            n = DenseVector.OfVector(n.Normalize(2));

            nx = n[0];
            ny = n[1];
            nz = n[2];

            theta = Math.Acos(Math.Cos(alpha1) * Math.Cos(alpha2));

            //Mount quaternion parts

            double q1 = Math.Cos(theta / 2);
            double q2 = nx * Math.Sin(theta / 2);
            double q3 = ny * Math.Sin(theta / 2);
            double q4 = nz * Math.Sin(theta / 2);

            DenseMatrix q = new DenseMatrix(1, 3, new double[] { q2, q3, q4 });
            DenseMatrix qT = new DenseMatrix(3, 1, new double[] { q2, q3, q4 });
            DenseMatrix I = DenseMatrix.CreateIdentity(3);
            DenseMatrix qS = new DenseMatrix(3);

            qS[0, 1] = -q4;
            qS[0, 2] = q3;
            qS[1, 2] = -q2;

            qS[1, 0] = q4;
            qS[2, 0] = -q3;
            qS[2, 1] = q2;

            DenseMatrix sub_RotationMatrix = (Math.Pow(q1, 2) - (q * qT)[0, 0]) * I + 2 * qT * q + 2 * q1 * qS;

            DenseMatrix temp_RotationMatrix = new DenseMatrix(nodesAmount*6);

            for(int i = 0; i < nodesAmount*6; i += 3)
            {
                temp_RotationMatrix.SetSubMatrix(i, i, sub_RotationMatrix);
            }

            RotationMatrix = temp_RotationMatrix;
        }

    }
}
