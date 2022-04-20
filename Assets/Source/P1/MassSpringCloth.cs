using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class MassSpringCloth : MonoBehaviour
{
    public class Node
    {
        public Vector3 pos;
        public Vector3 vel;
        public Vector3 force;

        public float mass;
        public float damping;

        public Fixer fixer; //guardo una referencia del fixer que afecta a este nodo en caso de tenerlo
        public bool isFixed; //determina si el nodo se encuentra estatico

        public bool isColliding;

        public Vector3 offset; //variable que almacena la distancia entre la posicion del nodo y el centro del fixer con el que colisiona
        public Vector3 windforce; //variable que acumula la fuerza de todos los vientos de la escena que afecten a la tela

        public MassSpringCloth manager;

        public Node(MassSpringCloth m, Vector3 position)
        {
            this.pos = position;
            this.vel = Vector3.zero;
            this.windforce = Vector3.zero;
            this.manager = m;
            this.isFixed = false;
            this.isColliding = false;
        }
        public void SetParams(float mass, float damping)
        {
            this.mass = mass;
            this.damping = damping;
        }
        public void ComputeForces()
        {
            force += mass * manager.Gravity - damping * vel + windforce;
        }
    }

    public class Spring
    {
        public string id; //Para diferencias los muelles de traccion y flexion

        public Node nodeA, nodeB;

        public float Length0; //distancia entre el nodoA y el nodoB en el instante inicial
        public float Length; //valor de la distancia entre ambos nodos en cada momento

        public float stiffness; //coeficiente de rigidez del muelle. A mayor rigidez mas gomosa sera la tela
        public float damping;

        public MassSpringCloth manager;

        public Spring(MassSpringCloth m, Node a, Node b, string id)
        {
            this.manager = m;
            this.nodeA = a;
            this.nodeB = b;
            this.id = id;
            this.Length = (nodeA.pos - nodeB.pos).magnitude;
            this.Length0 = this.Length;
        }

        public void SetParams(float stiffness, float damping)
        {
            this.stiffness = stiffness;
            this.damping = damping;
        }

        public void ComputeForces()
        {
            Vector3 u = nodeA.pos - nodeB.pos;
            Length = u.magnitude;
            u.Normalize();

            float stress = -stiffness * (Length - Length0) + stiffness * damping * Vector3.Dot(u, nodeA.vel - nodeB.vel);
            Vector3 force = stress * u;
            nodeA.force += force;
            nodeB.force -= force;
        }
    }

    public class Edge
    {
    //Clase Edge utilizada para simular y ordenar las aristas formadas por los nodos
        public int va, vb, other;

        public Edge(int a, int b, int o)
        {
            if (a < b)
            {
                va = a;
                vb = b;
            }
            else
            {
                vb = a;
                va = b;
            }
            other = o;
        }
    }

    public class EdgeComparer : IComparer<Edge>
    {
    //Clase EdgeComparer utilizada para ordenar segun el metodo Compare(Edge a, Edge b) que arista es menor aristas que sea iguales
    //entre si se situaran de forma consecutiva y mas adelante podemos aprovecharlas para crear los springs de flexion
        public int Compare(Edge a, Edge b)
        {
            if (a.va < b.va) return -1;
            else if (b.va < a.va) return 1;
            else if (a.vb < b.vb) return -1;
            else if (b.vb < a.vb) return 1;
            else return 0;
        }
    }

    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
    };

    public MassSpringCloth()
    {
        //Constructor para definir los valores por defecto
        this.Paused = true;
        this.TimeStep = 0.02f;
        this.Substeps = 1;
        this.Gravity = new Vector3(0.0f, -9.81f, 0.0f);
        this.IntegrationMethod = Integration.Symplectic;
    }

    #region InEditorVariables
    public Integration IntegrationMethod;
    public bool Paused;
    public float TimeStep;
    public int Substeps;
    public Vector3 Gravity;

    public float nodeMass;
    public float nodeDamping;
    public float springStiffnessTraccion;
    public float springStiffnessFlexion;
    public float springDamping;
    public float windFrictionCoeficient;

    public List<Node> nodes;
    public List<Spring> springs;
    public List<Edge> edges;

    public List<Fixer> fixers; //lista de fixer que afectan a la tela
    public List<Wind> winds; //lista de vientos que afectan a la tela
    public List<GameObject> collisions; //lista de objetos con los que la tela colisiona
    #endregion

    public void Start()
    {
        TimeStep = TimeStep / Substeps;

        nodes = new List<Node>();
        springs = new List<Spring>();
        edges = new List<Edge>();

        Mesh mesh = this.GetComponent<MeshFilter>().mesh; //malla de la tela
        Vector3[] vertices = mesh.vertices; //array de vertices de la malla de la tela
        int[] triangles = mesh.triangles; //array que almacena los indices de los nodos de cada triangulo

        createNodes(vertices);
        createSprings(triangles);
        fixNodes();
    }

    public void Update()
    {
        //Al pulsar "P" las fisicas de la tela se detendran
        if (Input.GetKeyUp(KeyCode.P))
            this.Paused = !this.Paused;

        Mesh mesh = this.GetComponent<MeshFilter>().mesh;  //malla de la tela
        Vector3[] vertices = new Vector3[mesh.vertexCount]; //array para almacenar las nuevas posiciones de los nodos calculadas en la fisica

        //Para cada nodo que se encuentra fixeado le actualizo la posicion a la de su fixer
        //De esta manera la mover o rotar el fixer los nodos se moveran junto a el
        foreach (Node node in nodes)
        {
            if (node.isFixed)
            {
                node.pos = node.fixer.transform.TransformPoint(node.offset);
            }
        }
        //reconvierto las posiciones globales a posiciones locales
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = transform.InverseTransformPoint(nodes[i].pos);
        }

        mesh.vertices = vertices; //igualo la posicion de los vertices de la malla a la de los nodos
    }

    public void FixedUpdate()
    {
        //Si esta pausado no continuo ejecutando
        if (this.Paused)
            return;

        //Dependiendo del metodo de integracion seleccionado se llamara a una u otra funcion de calculo de fisicas
        //La funcion se ejecuta por cada llamada al FixedUpdate Substeps veces.
        for(int i = 0; i < Substeps; i++)
        {
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.stepExplicit(); break;
                case Integration.Symplectic: this.stepSymplectic(); break;
                default:
                    throw new System.Exception("[ERROR] Should never happen!");
            }
        }
    }

    void createNodes(Vector3[] vertices)
    {
        //Creo los nodos en la posicion de cada vertice de la malla
        for (int i = 0; i < vertices.Length; i++)
        {
            nodes.Add(new Node(this, transform.TransformPoint(vertices[i])));
        }
    }
    void createSprings(int[] triangles)
    {
        //Creo los edges, los ordeno y creo a partir de ellos los spring tanto de flexion como de tranccion
        //Por cada triangulo, creo las 3 aristas uniendo los nodos correspondientes. Se crearan aristas duplicadas
        for (int i = 0; i < triangles.Length; i += 3)
        {
            edges.Add(new Edge(triangles[i + 0], triangles[i + 1], triangles[i + 2]));
            edges.Add(new Edge(triangles[i + 1], triangles[i + 2], triangles[i + 0]));
            edges.Add(new Edge(triangles[i + 2], triangles[i + 0], triangles[i + 1]));
        }

        //Creo un objeto de la clase EdgeComparer y ordeno la lista de arista mediante el comparador de esta clase haciendo que las aristas repetidas se situen consecutivamente
        EdgeComparer edgecomparer = new EdgeComparer();
        edges.Sort(edgecomparer);

        //Una vez ordenados las aristas, creo los muelles.
        for (int i = 0; i < edges.Count - 1; i++)
        {
            //Si la arista actual es igual a la siguiente arista se creara un muelle de flexion. En caso contrario, el muelle sera de traccion.
            if (edges[i].va == edges[i + 1].va && edges[i].vb == edges[i + 1].vb)
            {
                springs.Add(new Spring(this, nodes[edges[i].other], nodes[edges[i + 1].other], "flexion")); //creo los nodos de flexion
            }
            else
            {
                springs.Add(new Spring(this, nodes[edges[i].va], nodes[edges[i].vb], "traccion")); //creo los nodos de traccion
            }
        }
        //Debido a que en el bucle anterior recorro (edges.Count - 1) para evitar una excepcion OutboundException al comprobar cada arista con la siguiente, esto hace que el ultimo muelle no se cree.
        //Creo el ultimo muelle de la tela
        springs.Add(new Spring(this, nodes[edges[edges.Count - 1].va], nodes[edges[edges.Count - 1].vb], "traccion"));
    }
    void fixNodes()
    {
        //Fixea cada nodo que se encuentra en contacto con un fixer de la lista fixers
        foreach (Node node in nodes)
        {
            foreach (Fixer fixer in fixers)
            {
                if (fixer.CalculateCollision(node.pos))
                {
                    node.isFixed = true;
                    node.fixer = fixer;
                    node.offset = fixer.transform.InverseTransformPoint(node.pos); //almaceno la distancia entre el nodo y el centro del fixer
                }
            }
        }
    }
    private void stepExplicit()
    {
        computeWind();

        //Para cada nodo establezco la masa y el damping en tiempo real y calculo la fuerza correspondiente
        foreach (Node node in nodes)
        {
            node.SetParams(nodeMass, nodeDamping);
            node.force = Vector3.zero;
            node.ComputeForces();
        }

        //Para cada tipo de muelle establezco la rigidez y el damping y calculo la fuerza correspondiente
        foreach (Spring spring in springs)
        {
            if (spring.id == "traccion")
                spring.SetParams(springStiffnessTraccion, springDamping);
            else
                spring.SetParams(springStiffnessFlexion, springDamping);
            spring.ComputeForces();
        }

        //Para cada nodo que no esta fixeado calculo el movimiento
        foreach (Node node in nodes)
        {
            if (!node.isFixed)
            {
                node.pos += TimeStep * node.vel;
                node.vel += TimeStep / node.mass * node.force;
            }
        }
    }
    private void stepSymplectic()
    {
        computeWind();
        computeColisions();

        //Para cada nodo establezco la masa y el damping en tiempo real y calculo la fuerza correspondiente
        foreach (Node node in nodes)
        {
            node.SetParams(nodeMass, nodeDamping);
            node.force = Vector3.zero;
            node.ComputeForces();
        }

        //Para cada tipo de muelle establezco la rigidez y el damping y calculo la fuerza correspondiente
        foreach (Spring spring in springs)
        {
            if (spring.id == "traccion")
                spring.SetParams(springStiffnessTraccion, springDamping);
            else
                spring.SetParams(springStiffnessFlexion, springDamping);
            spring.ComputeForces();
        }

        //Para cada nodo que no esta fixeado calculo el movimiento
        foreach (Node node in nodes)
        {
            if (!node.isFixed)
            {
                if (!node.isColliding)
                {
                    node.vel += TimeStep / node.mass * node.force;
                    node.pos += TimeStep * node.vel;
                }
                if (node.isColliding)
                {
                    node.vel = TimeStep / node.mass * -node.force;
                    node.pos += TimeStep * node.vel;
                }
            }
        }
    }
    public void computeWind()
    {
        //Calculo la fuerza correspondiente a cada triangulo de la malla y se la reparto proporcionalmente a cada nodo
        Mesh mesh = this.GetComponent<MeshFilter>().mesh;
        int[] triangles = mesh.triangles;

        Vector3 windVel = Vector3.zero;

        //calculo la direccion del los vientos que afectan a la tela y su velocidad 
        foreach (Wind wind in winds)
        {
            if (wind.activate)
            {
                windVel = windVel + (wind.transform.forward * wind.strength);
            }
        }

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 triangleVel = (nodes[triangles[i + 0]].vel + nodes[triangles[i + 1]].vel + nodes[triangles[i + 2]].vel) / 3;

            //utilizo la formula de Heron para hayar el valor del area del triangulo
            float a = Vector3.Distance(nodes[triangles[i + 0]].pos, nodes[triangles[i + 1]].pos);
            float b = Vector3.Distance(nodes[triangles[i + 1]].pos, nodes[triangles[i + 2]].pos);
            float c = Vector3.Distance(nodes[triangles[i + 2]].pos, nodes[triangles[i + 0]].pos);

            float semiPrerimeter = (a + b + c) / 2;

            float triangleArea = (float)Math.Sqrt(semiPrerimeter * (semiPrerimeter - a) * semiPrerimeter * (semiPrerimeter - b) * semiPrerimeter * (semiPrerimeter - c));

            //Obtengo el vector normal del plano que contiene al triangulo
            Vector3 n = Vector3.Normalize(Vector3.Cross(nodes[triangles[i + 1]].pos - nodes[triangles[i + 0]].pos, nodes[triangles[i + 2]].pos - nodes[triangles[i + 0]].pos));

            //Calculo la fuerza mediante la formula siguiente
            Vector3 windForce = windFrictionCoeficient * triangleArea * Vector3.Dot(n, windVel - triangleVel) * n;

            //Reparto a cada nodo la fuerza de viento correspondiente
            nodes[triangles[i + 0]].windforce = windForce / 3;
            nodes[triangles[i + 1]].windforce = windForce / 3;
            nodes[triangles[i + 2]].windforce = windForce / 3;
        }
    }
    public void computeColisions()
    {
        foreach(Node node in nodes)
        {
            foreach (GameObject colision in collisions)
            {
                Bounds bounds = colision.GetComponent<Collider>().bounds; //almaceno los limites del collider del objeto
                
                if (bounds.Contains(node.pos))
                {
                    node.isColliding = true;
                }
                else
                {
                    node.isColliding = false;
                }
            }
        }
    }
}
