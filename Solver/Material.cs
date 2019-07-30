namespace Solver
{
    class Material
    {
        public string name; // Material's name
        private double poisson; // Poisson Coefficient 
        public double E { get; private set; } // Young's Modulus
        public double G { get; private set; } // Shear Modulus

        public Material(double E, double poisson, string name = null)
        {
            this.E = E;
            this.poisson = poisson;
            this.G = E / (2 + 2*poisson);
            this.name = name;
        }

    }
}
