using System;

namespace Solver
{
    class Section
    {
        public double Area { get; protected set; }
        public double InertiaY { get; protected set; }
        public double InertiaZ { get; protected set; }
        public double J { get; protected set; }

        public Section() { }
        public Section(double Area, double InertiaY, double InertiaZ, double J)
        // Generic Sections contructor, where all parameters are supplied directly
        {
            this.Area = Area;
            this.InertiaY = InertiaY;
            this.InertiaZ = InertiaZ;
            this.J = J;
        }

    }

    class RectangularSection : Section
    {
        public RectangularSection(double width, double height)
        {
            this.Area = width*height;
            this.InertiaY = height*width*width*width / 12;
            this.InertiaZ = width*height*height*height / 12;
            this.J = this.InertiaY + this.InertiaZ;            
        }

    }

    class CircularSection : Section
    {
        public CircularSection(double radius)
        {
            this.Area = Math.PI*radius*radius;
            this.InertiaY = this.InertiaZ = Math.PI*Math.Pow(radius, 4) / 4;
            this.J = InertiaY + InertiaZ;
        }

    }

}
