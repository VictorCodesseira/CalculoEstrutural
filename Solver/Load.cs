using MathNet.Numerics.LinearAlgebra.Double;

namespace Solver
{
    class Load
    {
        // Load is represented by a 6 element vector, with each element being the 
        // magnitude of the load in each cartesian axis(translational and rotational)
        public DenseVector FVector;
        public Load()
        {
            this.FVector = new DenseVector(6);
        }
        public Load(double[] FVector)
        {
            this.FVector = new DenseVector(FVector);
        }
        public void Divide(double value)
        {
            this.FVector = (DenseVector)this.FVector.Divide(value);
        }
    }

    class DistributedLoad
    {
        // Distributed Loads are represented in the same way, but with a start and an end value
        public double[] startValue, endValue;
        public DistributedLoad(double[] startValue, double[] endValue)
        {
            this.startValue = startValue;
            this.endValue = endValue;
        }
    }

}

    
