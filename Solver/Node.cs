﻿using MathNet.Numerics.LinearAlgebra.Double;
using Newtonsoft.Json;
using System;

namespace Solver
{
    class Node
    {
        public int ID; // Node unique identifier

        [JsonIgnore]
        public Load load; // Load that is applied directly in the node

        [JsonIgnore]
        public Link link; // Link that locks some DOFs in place

        [JsonIgnore]
        public bool hinge = false;

        private double x, y, z;

        public double[] ForcedDisplacements = { 0, 0, 0, 0, 0, 0 };

        [JsonIgnore]
        public Vector3D Position // Struct with the 3 cartesian coordinates of the Node
        {
            get
            {
                return new Vector3D(x, y, z);
            }
        }

        public DenseVector NodalDisplacements; // Displacements of the node, in the global cartesian system
        public DenseVector NodalReactions; // Reactions of the node, in the global cartesian system

        public Node(double x, double y, double z, int ID)
        {
            this.ID = ID;
            this.x = x;
            this.y = y;
            this.z = z;
            this.load = new Load();
            this.link = new Link();
            this.NodalDisplacements = new DenseVector(6);
            this.NodalReactions = new DenseVector(6);
        }

        public void addLoad(Load ld)
        {
            this.load = ld;
        }

        public void addLink(Link lk)
        {
            this.link = lk;
        }

        public void addForcedDisplacement(double dx, double dy, double dz)
        {
            this.ForcedDisplacements[0] = dx;
            this.ForcedDisplacements[0] = dy;
            this.ForcedDisplacements[0] = dz;
        }

        public void addForcedRotation(double drx, double dry, double drz)
        {
            this.ForcedDisplacements[0] = drx;
            this.ForcedDisplacements[0] = dry;
            this.ForcedDisplacements[0] = drz;
        }

        public void addHinge()
        {
            this.hinge = true;
        }

        public double Distance(Node other)
        {
            return Math.Sqrt((this.x - other.x) * (this.x - other.x) + (this.y - other.y) * (this.y - other.y) + (this.z - other.z) * (this.z - other.z));
        }


    }

    struct Vector3D
    {
        public double x, y, z;
        public Vector3D(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }
}
