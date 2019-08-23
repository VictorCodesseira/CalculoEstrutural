using System;
using System.Reflection;

namespace Solver
{
    static public class Solver
    {
        static public void Calculate(string input_filename, string output_filename = "")
        {
            BarSystem Sys = JsonInterface.ReadInput(input_filename);
            Sys.Solve();
            if (output_filename == "")
            {
                string[] words = input_filename.Split('.');
                for (int i = 0; i < words.Length - 1; i++)
                {
                    output_filename = (i != 0) ? (output_filename + ".") : output_filename;
                    output_filename += words[i];
                }
                output_filename += "_output.json";
            }
            JsonInterface.WriteOutput(Sys, output_filename);

        }

        static void RunTests()
        {
            Type MainClass = typeof(Solver);
            MethodInfo[] methods = MainClass.GetMethods(BindingFlags.DeclaredOnly | BindingFlags.Static | BindingFlags.NonPublic);
            // Gets the name of every method in the class Solver
            for (int i = 0; i < methods.Length; i++)
            {
                MethodInfo method = (MethodInfo)methods[i];
                if (method.Name.StartsWith("test_")) { // If the routine name starts with "test_", call it
                    method.Invoke(null, null);
                }
            }
        }

        // Test Routines
        static void test_StiffnessMatrix()
        {
            Material mat1 = new Material(1, 1);
            Section sect1 = new RectangularSection(1, 1);
            Node node1 = new Node(0, 0, 0, 0);
            Node node2 = new Node(1, 0, 0, 1);
            Beam testBeam = new Beam(node1, node2, mat1, sect1, 0);

            /*
            double[,] controlArray = {{1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0 },
                                      {0, 1, 0, 0, 0, 0.5, 0, -1, 0, 0, 0, 0.5 },
                                      {0, 0, 1, 0, -0.5, 0, 0, 0, -1, 0, -0.5, 0 },
                                      {0, 0, 0,  },
                                      { },
                                      { },
                                      { },
                                      { },
                                      { },
                                      { },
                                      { },
                                      { } }
            DenseMatrix controlMatrix = new DenseMatrix();
            DenseMatrix stiffnessMatrix = testBeam.LocalStiffnessMatrix;
            */
        }

        static void test_ConnectionVector()
        {

        }

        static void test_NodalForcesVector()
        {

        }

        static void test_RotationMatrix()
        {

        }

        static void test_SystemStiffnessMatrix()
        {

        }

        static void test_SystemForcesVector()
        {

        }

        static void test_Solution()
        {

        }

        static void test_Stresses()
        {

        }

        static void test_ElasticLines()
        {

        }

    }

}
