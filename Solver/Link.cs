namespace Solver
{
    class Link
    {
        public int[] restrictions = { 0, 0, 0, 0, 0, 0 }; // restrictions[n] = 1 means that the nth DOF is blocked

        public Link() { }
        public Link(int[] restrictions) // Generic Link constructor
        {
            for (int i = 0; i < 6; i++)
            {
                this.restrictions[i] = restrictions[i];
            }
        }
    }

    class Mount : Link
    {
        // Mount link, blocks all DOFs
        public Mount()
        {
            for (int i = 0; i < 6; i++)
            {
                this.restrictions[i] = 1;
            }
        }
    }


    class Support : Link
    {
        // Support Link, blocks a single translational DOF, in the "direction" received
        public Support(int direction) // 0 = x, 1 = y, 2 = z
        {
            this.restrictions[direction] = 1;
        }
    }

    class Fixed_Support : Link
    {
        // Fixed Support link, blocks all translational DOFs and no rotational ones
        public Fixed_Support()
        {
            for (int i = 0; i <3; i++)
            {
                this.restrictions[i] = 1;
            }
        }
    }

    class Ring : Link
    {
        // Ideal Ring link, blocks 2 translational DOFs, that are not in the "direction" received
        public Ring(int direction) // 0 = x, 1 = y, 2 = z
        {
            for(int i = 0; i < 3; i++)
            {
                this.restrictions[i] = 1;
            }
            this.restrictions[direction] = 0;
        }
    }
}
