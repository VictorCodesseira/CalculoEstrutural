using Newtonsoft.Json;
using System.Collections.Generic;
using System.IO;
using System;

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
                double E = (double)mat["E"];
                double Poisson = (double)mat["Poisson"];
                string Name = (string)mat["Name"];
                materials.Add(Name, new Material(E, Poisson, Name));
            }

            foreach (dynamic sect in json["Sections"]) // Create instances of sections
            {
                int sectID = (int)sect["ID"];
                if ((string)sect["Type"] == "Rectangular")
                {
                    double width = (double)sect["Width"];
                    double height = (double)sect["Height"];
                    sections[sectID] = new RectangularSection(width, height);
                }
                else if ((string)sect["Type"] == "Circular")
                {
                    double radius = (double)sect["Radius"];
                    sections[sectID] = new CircularSection(radius);
                } else
                {
                    double area = (double)sect["Area"];
                    double IY = (double)sect["InertiaY"];
                    double IZ = (double)sect["InertiaZ"];
                    double J = (double)sect["PolarInertia"];
                    sections[sectID] = new Section(area, IY, IZ, J);
                }
            }

            foreach (dynamic nd in json["Nodes"]) // Create instances of nodes
            {
                int nodeID = (int)nd["ID"];
                double x = (double)nd["x"];
                double y = (double)nd["y"];
                double z = (double)nd["z"];
                nodes[nodeID] = new Node(x, y, z, nodeID);
            }

            try
            {
                foreach (dynamic bm in json["Beams"]) // Create instances of beams, based on start and end nodes, section and materials
                {
                    int beamID = (int)bm["ID"];
                    int startNodeID = (int)bm["StartNode"];
                    int endNodeID = (int)bm["EndNode"];
                    string materialName = (string)bm["Material"];
                    int sectID = (int)bm["Section"];
                    elements[beamID] = new Beam(nodes[startNodeID], nodes[endNodeID], materials[materialName], sections[sectID], beamID);
                }
            }
            catch (NullReferenceException)
            {
                Console.WriteLine("No regular beam");
            }

            try
            {
                foreach (dynamic cbm in json["CurvedBeams"]) // Create instances of curved beams, based on start, center and end nodes, section and materials
                {
                    int beamID = (int)cbm["ID"];
                    int startNodeID = (int)cbm["StartNode"];
                    int centerNodeID = (int)cbm["CenterNode"];
                    int endNodeID = (int)cbm["EndNode"];
                    string materialName = (string)cbm["Material"];
                    int sectID = (int)cbm["Section"];
                    CurvedBeam cbeam = new CurvedBeam(nodes[startNodeID], nodes[endNodeID], nodes[centerNodeID], materials[materialName], sections[sectID], beamID);
                    elements[beamID] = cbeam;
                    nodes[centerNodeID] = cbeam.middleNode;
                }
            }
            catch (NullReferenceException)
            {
                Console.WriteLine("No Curved Beams");
            }

            try
            {
                foreach (dynamic ld in json["Loads"]) // Add loads to nodes
                {
                    int nodeID = (int)ld["Node"];
                    double[] vector = new double[6];
                    ld["Forces"].ToObject<double[]>().CopyTo(vector, 0);
                    ld["Moments"].ToObject<double[]>().CopyTo(vector, 3);
                    nodes[nodeID].addLoad(new Load(vector));
                }
            }
            catch (NullReferenceException)
            {
                Console.WriteLine("No point loads");
            }

            try
            {
                foreach (dynamic dld in json["DistributedLoads"]) // Add distributed loads to beams
                {
                    int beamID = (int)dld["Beam"];
                    double[] startVector = dld["StartVector"].ToObject<double[]>();
                    double[] endVector = dld["EndVector"].ToObject<double[]>();
                    (elements[beamID] as Beam).addLoad(new DistributedLoad(startVector, endVector));
                }
            }
            catch (NullReferenceException)
            {
                Console.WriteLine("No distributed loads");
            }

            Dictionary<string, int> directions = new Dictionary<string, int>();
            directions["x"] = 0;
            directions["y"] = 1;
            directions["z"] = 2;
            foreach (dynamic lk in json["Links"]) // Add links to nodes
            {
                int node = (int)lk["Node"];
                string type = (string)lk["Type"];
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
                        dir = (string)lk["Direction"];
                        link = new Support(directions[dir]);
                        break;
                    case "Ring":
                        dir = (string)lk["Direction"];
                        link = new Ring(directions[dir]);
                        break;
                    default:
                        int[] restrictions = lk["Vector"].ToObject<int[]>();
                        link = new Link(restrictions);
                        break;

                }
                nodes[node].addLink(link);
            }

            try
            {
                foreach (dynamic hin in json["Hinges"]) // Add hinges to nodes
                {
                    int nodeID = (int)hin["Node"];
                    Node releaseStartNode = nodes[nodeID];
                    Node releaseEndNode = new Node(releaseStartNode.Position.x, releaseStartNode.Position.y, releaseStartNode.Position.z, nodes.Count);
                    nodes[nodes.Count] = releaseEndNode;
                    foreach(Element el in elements.Values)
                    {
                        if (el.startNode.ID == nodeID)
                        {
                            el.startNode = releaseEndNode;
                            break;
                        } else if (el.endNode.ID == nodeID)
                        {
                            el.endNode = releaseEndNode;
                            break;
                        }
                    }
                    elements[elements.Count] = new Release(releaseStartNode, releaseEndNode, elements.Count);
                }
            }
            catch (NullReferenceException)
            {
                Console.WriteLine("No hinges");
            }

            try
            {
                foreach (dynamic fd in json["ForcedDisplacements"]) // Add displacements to nodes
                {
                    int node = (int)fd["Node"];
                    double[] disp = fd["Displacements"].ToObject<double[]>();
                    double[] rot = fd["Rotations"].ToObject<double[]>();
                    nodes[node].addForcedDisplacement(disp[0], disp[1], disp[2]);
                    nodes[node].addForcedRotation(rot[0], rot[1], rot[2]);
                }
            }
            catch (NullReferenceException)
            {
                Console.Write("No forced displacements");
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
