using Newtonsoft.Json;
using System.Collections.Generic;
using System.IO;

namespace Solver
{
    static class JsonInterface
    {
        static public BarSystem ReadInput(string Filename)
        {
            Dictionary<string, Material> materials = new Dictionary<string, Material>(); // Dict with all materials and name as index
            Dictionary<int, Section> sections = new Dictionary<int, Section>(); // Dict with all sections and ID as index
            Dictionary<int, Node> nodes = new Dictionary<int, Node>(); // Dict with all nodes and ID as index
            Dictionary<int, Element> elements = new Dictionary<int, Element>(); // Dict with all elements and ID as index

            string file = File.ReadAllText(Filename);
            dynamic json = JsonConvert.DeserializeObject(file); // Convert file into a dynamic object

            foreach (dynamic mat in json["Materials"]) // Create instances of materials
            {
                double E = mat["E"];
                double Poisson = mat["Poisson"];
                string Name = mat["Name"];
                materials.Add(Name, new Material(E, Poisson, Name));
            }

            foreach (dynamic sect in json["Sections"]) // Create instances of sections
            {
                int sectID = sect["ID"];
                if (sect.Type == "Rectangular")
                {
                    double width = sect["Width"];
                    double height = sect["Height"];
                    sections[sectID] = new RectangularSection(width, height);
                }
                else if (sect.Type == "Circular")
                {
                    double radius = sect["Radius"];
                    sections[sectID] = new CircularSection(radius);
                } else
                {
                    double area = sect["Area"];
                    double IY = sect["InertiaY"];
                    double IZ = sect["InertiaZ"];
                    double J = sect["PolarInertia"];
                    sections[sectID] = new Section(area, IY, IZ, J);
                }
            }

            foreach (dynamic nd in json["Nodes"]) // Create instances of nodes
            {
                int nodeID = nd["ID"];
                double x = nd["x"];
                double y = nd["y"];
                double z = nd["z"];
                nodes[nodeID] = new Node(x, y, z, nodeID);
            }

            foreach (dynamic bm in json["Beams"]) // Create instances of beams, based on start and end nodes, section and materials
            {
                int beamID = bm["ID"];
                int startNodeID = bm["Start Node"];
                int endNodeID = bm["End Node"];
                string materialName = bm["Material"];
                int sectID = bm["Section"];
                elements[beamID] = new Beam(nodes[startNodeID], nodes[endNodeID], materials[materialName], sections[sectID], beamID);
            }

            foreach (dynamic cbm in json["Curved Beams"]) // Create instances of curved beams, based on start, center and end nodes, section and materials
            {
                int beamID = cbm["ID"];
                int startNodeID = cbm["Start Node"];
                int centerNodeID = cbm["Center Node"];
                int endNodeID = cbm["End Node"];
                string materialName = cbm["Material"];
                int sectID = cbm["Section"];
                elements[beamID] = new CurvedBeam(nodes[startNodeID], nodes[endNodeID], nodes[centerNodeID], materials[materialName], sections[sectID], beamID);
            }

            foreach (dynamic ld in json["Loads"]) // Add loads to nodes
            {
                int nodeID = ld["Node"];
                double[] vector = new double[6];
                ld["Forces"].ToObject<double[]>().CopyTo(vector, 0);
                ld["Moments"].ToObject<double[]>().CopyTo(vector, 3);
                nodes[nodeID].addLoad(new Load(vector));
            }

            foreach (dynamic dld in json["Distributed Loads"]) // Add distributed loads to beams
            {
                int beamID = dld["Beam"];
                double[] startVector = dld["Start Vector"].ToObject<double[]>();
                double[] endVector = dld["End Vector"].ToObject<double[]>();
                (elements[beamID] as Beam).addLoad(new DistributedLoad(startVector, endVector));
            }

            Dictionary<string, int> directions = new Dictionary<string, int>();
            directions["x"] = 0;
            directions["y"] = 1;
            directions["z"] = 2;
            foreach (dynamic lk in json["Links"]) // Add links to nodes
            {
                int node = lk["Node"];
                string type = lk["Type"];
                string dir;
                Link link;
                switch (type)
                {
                    case "Mount":
                        link = new Mount();
                        break;
                    case "Fixed Support":
                        link = new Fixed_Support();
                        break;
                    case "Support":
                        dir = lk["Direction"];
                        link = new Support(directions[dir]);
                        break;
                    case "Ring":
                        dir = lk["Direction"];
                        link = new Ring(directions[dir]);
                        break;
                    default:
                        int[] restrictions = lk["Vector"].ToObject<int[]>();
                        link = new Link(restrictions);
                        break;

                }
                nodes[node].addLink(link);
            }

            foreach (dynamic hin in json["Hinges"]) // Add hinges to nodes
            {
                int beam = hin["Beam"];
                int node = hin["Node"];
                ((Beam)elements[beam]).addHinge(node);
            }

            foreach (dynamic fd in json["Forced Displacements"]) // Add displacements to nodes
            {
                int node = fd["Node"];
                double[] disp = fd["Displacements"].ToObject<double[]>();
                double[] rot = fd["Rotations"].ToObject<double[]>();
                nodes[node].addForcedDisplacement(disp[0], disp[1], disp[2]);
                nodes[node].addForcedRotation(rot[0], rot[1], rot[2]);
            }

            Node[] nodes_array = new Node[nodes.Count]; 
            nodes.Values.CopyTo(nodes_array, 0); // Convert nodes dict to array

            
            Element[] elements_array = new Element[elements.Count]; // Join all types of elements
            elements.Values.CopyTo(elements_array, 0); // Convert nodes dict to array

            BarSystem sys = new BarSystem(nodes_array, elements_array); // Create system from beams and nodes array
            return sys;

        }

        static public void WriteOutput(BarSystem sys, string filename)
        {
            // Converts the system to its Json representation, based on the [JsonIgnore] attributes
            string output = JsonConvert.SerializeObject(sys, Formatting.Indented);
            File.WriteAllText(filename, output); // WriteFile
        }
    }
}
