/* Graph.h 'barebones' version provided by John Lillis, CS251 F18 */
/* Modified for CS401 F19 by Frank Errichiello & Tomasz Hulka */


#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <sstream>
#include <fstream>

using std::string;
using std::vector;
using std::unordered_map;
using std::unordered_set;

#define UNDISCOVERED 'u'
#define DISCOVERED   'd'
#define ACTIVE       'a'
#define FINISHED     'f'
#define INF          1000000000000

/*
 * function:  pvec
 * description:  utility function that prints the elements of
 *   a vector: one per line.
 * 
 * Note that this is a templated function; only works if the type
 *   T is acceptable with:
 *
 *     cout << var_of_type_T
 */
template <typename T>
void pvec(const std::vector<T> & vec) {

  for(const T &x : vec) {
    std::cout << x << "\n";;
  }
}


/*
 * class:  graph
 *
 * desc:   class for representing directed graphs.  Uses the
 *   adjacency list representation.
 *
 */

class graph {

  private:

    // note:  this struct does not store both
    //   vertices in the edge -- just one.  This
    //   is because of the overall structure of
    //   the adjacency list organization:  an
    //   edge struct is stored in a vector associated
    //   with the other vertex.
    struct edge {
      int vertex_id;
      double time;
      double cost;
      vector<int> path; // an additional piece of bookkeaping that is used only for the CCSSSP algorithm

      edge ( int vtx_id=0, double _cost = 0.0, double _time=1.0) 
        : vertex_id { vtx_id}, time { _time}, cost { _cost}
      { }
    };

    // a vertex struct stores all info about a particular
    //    vertex:  name, ID, incoming and outgoing edges.
    struct vertex {
      int id;
      vector<edge> outgoing;
      vector<edge> incoming;
      string name;

      vertex ( int _id=0, string _name="") 
        : id { _id }, name { _name } 
      { }
    };

    // _name2id:
    // Each vertex in a graph is identified in two ways:
    //      - by its unique 'name' which is a string (so things are
    //        friendly to the outside world).
    //      - by its unique integer ID which is more convenient 
    //        internally.  If a graph has N vertices, the 
    //        corresponding IDs are ALWAYS 0..N-1.
  
    unordered_map<string, int> _name2id;

    // vertices:
    //   vertices is the primary data structure:  it is an  implementation
    //      of a pretty standard adjacency list.
    //   It is indexed by vertex ID.
    //   vertices[u] contains everything we need to know about vertex u:
    //       - name (string)
    //       - ID (int).  Somewhat redundant since vertices[u].id == u
    //       - outgoing edges (as a vector of edge structures)
    //       - incoming edges (as a vector of edge structures)
    //       
    //   See struct vertex above
    vector<vertex> vertices;

    // the unordered set edges isn't going to be of much interest
    //   to you.  Its main purpose is to detect duplicate edges
    //   while building a graph (see add_edge)..
   
    unordered_set<string> edges;



  public:

    // this struct is used for capturing the results of an operation.
    // typically a "report" will be a vector of vertex_labels indexed
    // by vertex-id.
    struct vertex_label {
      double dist;
      int pred;
      char state;
      int npaths;
      
      vertex_label( double _dist=0.0, int _pred=-1, char _state='?',
          int _npaths=0) 
        : dist { _dist }, pred { _pred }, state { _state}, npaths { 0 }
      { }

    };

    
    // a struct holding an 'instance' of the CCSSSP algorithm run on the given graph
    struct ccSPBundle {
      vector<vector<std::pair<double, double>>> S;
      vector<vector<vector<int>>> pred;
      int source;
      int destination;
      double budget;
      
    };
  

    graph() {}

    ~graph() {}

  private:

    int add_vertex(const string &name) {
      int id = vertices.size();
        vertices.push_back(vertex(id, name));
        _name2id[name] = id;
        return id;
    }


    /*
     * function:  edge_string
     *
     * returns concatenation of src and dest vertex strings with
     * a single space between
     *
     * Purpos:  gives a unique string representing the edge
     * -- data member edges stores sets of such strings to
     * quickly detect if an edge has already been created.
     *
     */
    static
    string edge_string(const string &src, const string &dest) {
      return src + " " + dest;
    }


    /*
     * function: p_edge
     * desc:  simple function for printing an edge
     */
    void p_edge(edge &e) {
      std::cout << "(" << id2name(e.vertex_id) 
        << ", " << e.cost << ", " << e.time << ") ";
    }

  public:

    /*
     * func:  id2name
     * desc:  returns vertex name (a string) associated with given 
     *         vertex id.
     *
     *         If id not valid for given graph, the string "$NONE$"
     *         is returned.
     */
    string  id2name(int id) {
      if(id<0 || id>=vertices.size())
        return "$NONE$";
      return vertices[id].name;
    }

    /*
     * func: name2id
     * desc: returns integer vertex id of given vertex name.
     *       If there is no such vertex in the graph, -1 is returned.
     */
    int name2id(const string &vtx_name) {
      if(_name2id.count(vtx_name)==0)
        return -1;
      return _name2id[vtx_name];
    }

    /*
     * func: name_vec2string
     * desc: utility function - if you have a bunch of
     *   vertex names (as strings) stored in a vector, this
     *   function puts the names in a single string with
     *   nodes separated by single spaces.
     *
     *   Might be handy for things like getting an easy to
     *   print representation of a path for example.
     */
    string name_vec2string(const vector<string> &vec) {
      string s = "";
      int i;

      if(vec.size()==0)
        return s;

      s = s + vec[0];
      for(i = 1; i<vec.size(); i++) {
        s = s + " " + vec[i];
      }
      return s;
    }

    /*
     * func: id_vec2string
     * desc: utility function - if you have a bunch of
     *   vertex ids (ints) stored in a vector, this
     *   function converts them to names and builds a in a 
     *   single string with nodes-names separated by single spaces.
     *
     *   Might be handy for things like getting an easy to
     *   print representation of a path for example.
     */
    string id_vec2string(const vector<int> &vec) {
      string s = "";
      int i;

      if(vec.size()==0)
        return s;

      
      s = s + "[" + id2name(vec[0]);
      for(i = 1; i<vec.size(); i++) {
        s = s + ", " + id2name(vec[i]);
      }
      return s + "]";
    }


    /*
     * func: add_edge
     * desc: adds edge (src,dest) with given weight to graph if
     *   possible.
     *
     *       If edge (src,dest) is already in graph, the graph is
     *       unchanged and false is returned.
     *
     *       Otherwise the edge is added and true is returned.
     *
     *       Note:  if src and/or dest are not currently vertices
     *         in the graph, they will be added.
     */
    bool add_edge(const string &src, const string &dest, 
        double cost=1.0, double time = 0.0) {

      int s_id, d_id;

      string estring = edge_string(src, dest);

      if(edges.count(estring)==1) {
        std::cerr << "warning: duplicate edge '"
          << estring << "'\n";
        return false;
      }

      edges.insert(estring);

      // get id for source vertex
      if(_name2id.count(src)==0) 
        s_id = add_vertex(src);
      else
        s_id = _name2id[src];

      // get id for destination vertex
      if(_name2id.count(dest)==0) 
        d_id = add_vertex(dest);
      else
        d_id = _name2id[dest];

      vertices[s_id].outgoing.push_back(edge(d_id, cost, time));
      vertices[d_id].incoming.push_back(edge(s_id, cost, time));

      return true;
    }


    /*
     * func: add_edge(string &)
     * desc: takes an edge specification as a single string, 
     *   parses the string into src vertex, dest vertex and
     *   weight (optional).
     *
     *   If parsing is successful, add_edge(string, string, double) above
     *   is called to do the "real work".
     *
     * returns true on success; false on failure (parse error or
     *   call to add_edge failed).
     *
     * expected format:
     *
     *   the given string must have either two or three tokens (exactly).
     *
     *   If it has three tokens, the third token must be parseable as
     *   a double.
     */
    bool add_edge(const string &str) {
      std::stringstream ss(str);
      string src, dest, junk, cost_str, time_str;
      double cost, time;

      if(!(ss >> src))
        return false;
      if(!(ss >> dest))
        return false;
      if(!(ss >> cost_str)){
        // only two tokens: use default weight 
        cost = 1.0;
      }

      if (!(ss >> time_str)) {
        // only three tokens: use default cost
        time = 0.0;

      }
      else {
        if(!(std::stringstream(cost_str) >> cost)){
          // couldn't parse cost
          return false;
        }

        if(!(std::stringstream(time_str) >> time)){
          // couldn't parse time
          return false;
        }

        if(ss >> junk){
          // extra token?  format error
          return false;
        }
      }

      add_edge(src, dest, cost, time);

      return true;
    }

    void _add_edge(const string &str) {

      if(!add_edge(str))
        std::cout << "add_edge failed; str='" <<
          str << "'\n";
    }

    /* 
     * func: read_file
     * desc: reads given file (if possible) as a 
     *   sequence of edges -- one edge per line.
     *   The first line is the number of vertices
     *   Each line except the first is expected to be in the form:
   
           <source-vertex> <dest-vertex> {<cost>} {<time>}
     *
     * where the vertices are given as strings and
     *   the edge weight and cost is a number (read as a double).
     * The edge weight and cost is optional (indicated by {}).
     * 
     * Ex: an edge from Chicago to NewYork with cost 201.9, time 150.0: 
       Chicago NewYork 201.9 150.0

     * if no weight is specified, the edge defaults to a cost
     *   of 1.0 and time of 0.0
     */
    bool read_file(const string &fname) {
      std::ifstream file;
      string line;

      file.open(fname, std::ios::in);
      if(!file.is_open())
        return false;
      

      bool firstEdge = false; //keeps track of valid lines so that the first line can be parsed differently
      
      while(getline(file, line)) {
        // skip blank lines
        if(line.length() > 0) {
          
          if (!firstEdge) {
            // we have a valid line and it must be the number of vertices
            int numV = stoi(line);

            for (int i = 0; i < numV; i++) {
              add_vertex(std::to_string(i));
            }
            
            firstEdge = true;
          }
         
          else if(!add_edge(line)) {
            std::cerr << "warning: skipped input line '" 
              << line << "' (ill-formatted)\n";
          }

          
        }

      }
      file.close();
      
      return true;
    }


    int num_nodes() {
      return vertices.size();
    }
    int num_edges() {
      return edges.size();
    }

  private:
    void init_report(std::vector<vertex_label> & report) {
      int u;

      report.clear();
      for(u=0; u<vertices.size(); u++) {
        report.push_back(vertex_label(-1, -1, UNDISCOVERED));
      }
    }


  private:

    // given vertices u and v, return the edge between them (assume caller always knows an edge exists)
    const edge getEdge(int u, int v) {
      
      for (const edge &e : vertices[u].outgoing) {
        if ( e.vertex_id == v) {
          return e;
        }
      }
    }
    

    // for dijkstra
    void relax(int u, int v, vector<double> &d, vector<int> &pred) {

      edge e = getEdge(u,v);
      if (d[v] > d[u] + e.cost) {
        d[v] = d[u] + e.cost;
        pred[v] = u;
      }

    }

    // used for ordering the min-heap
    class edgeCompare { 
    public: 
        int operator() (const edge& e1, const edge& e2) 
        { 
          if (e1.cost == e2.cost)
            return e1.time > e2.time;
          return e1.cost > e2.cost;
        } 
    }; 
  

    vector<int> _dijkstra(int src, vector<vertex_label> &report) {
      
      edge u;

      //init_ss 
      vector<int> pred(vertices.size(), -1);
      vector<double> d(vertices.size(), INF);

      vector<edge> S;
      std::priority_queue<edge, vector<edge>, edgeCompare> Q;

      

    
      for(int u=0; u< vertices.size(); u++) {
        for(edge &e : vertices[u].outgoing) 
          Q.push(e);
      }
     


      d[src] = 0;
      Q.push(edge(src, 0, 0));
      

      init_report(report);

      while(!Q.empty()) {

        u = Q.top();
        Q.pop();

        S.push_back(u);

        for (edge &v : vertices[u.vertex_id].outgoing) 
          relax(u.vertex_id, v.vertex_id, d, pred);
        
      }

      return pred;
    
    }


    void _costConstrained(ccSPBundle &answer) {


      edge u;
      int src = answer.source;

      vector<vector<std::pair<double, double>>> S(vertices.size(), vector<std::pair<double, double>>());
      std::priority_queue<edge, vector<edge>, edgeCompare> Q;

      // this min-heap uses 'edges' to keep track of cost,time bundles
      // an 'edge' has been modified to store a path (vector of ints)
      

      vector<vector<vector<int>>> path(vertices.size(), vector<vector<int>>());

      S[src].push_back(std::pair<double, double>(0,0));
      edge init(src, 0, 0);
      init.path = vector<int>(1, src);
      Q.push(init);
      
      

      while(!Q.empty()) {

        u = Q.top();
        Q.pop();

        for (edge &v : vertices[u.vertex_id].outgoing) {
          edge e = getEdge(u.vertex_id,v.vertex_id);
          if (S[v.vertex_id].size() == 0 || S[v.vertex_id][S[v.vertex_id].size() -1].second > e.time) {
            edge tmp(v.vertex_id, u.cost + e.cost, u.time + e.time);
            tmp.path = u.path;
            tmp.path.push_back(v.vertex_id);
            Q.push(tmp);

          }
        }

      
        if (S[u.vertex_id].size() == 0 || S[u.vertex_id][S[u.vertex_id].size() - 1].second > u.time) {
            S[u.vertex_id].push_back(std::pair<double,double>(u.cost, u.time));
            path[u.vertex_id].push_back(u.path);
            

        }
    

        
        
      }


      answer.S = S;
      answer.pred = path;
      return;
    }
  
  public:

    void costConstrained(ccSPBundle &answer) {

      int src = answer.source;
      int dest = answer.destination;

      // only run the algorithm if we have a valid problem instance
      if(!(src < 0 || src >= num_nodes() || dest < 0 || dest >= num_nodes())) 
        _costConstrained(answer);
      
      disp_report(answer); // displays final report

      return;

    }

    bool dijkstra(int src, vector<vertex_label> & rpt) {

      if(src < 0 || src >= num_nodes()) 
        return false;

      _dijkstra(src, rpt);

      
      return true;
      
    }

  
    void disp_paths(const ccSPBundle &rpt) {

      std::cout << "\tnon-dominated paths:\n";
      for (const vector<int> &p : rpt.pred[rpt.destination]) {
        std::cout << "\t" << id_vec2string(p) << "\n";
      }
        
    }


    // outputs the fastest cost-feasible path
    void disp_fastest(const ccSPBundle &rpt) {

      
      bool feasible = false; // stores whether or not there is a path s -> d with budget b
      std::pair<double, double> best;

      // at the end of this loop, we'll have a path with the shortest time constrained to our budget
      int index = -1;
      for (const std::pair<double,double> &p : rpt.S[rpt.destination]) {
        if (p.first <= rpt.budget) {
          feasible = true;
          best = p;
          index++;
        }
        
      }

      if (feasible) {
        std::cout << "\n\tFastest cost-feasible path from " << rpt.source 
                  << " to " << rpt.destination << ":\n"
                  << "\t" << id_vec2string(rpt.pred[rpt.destination][index]) //prints the path
                  << "\n\twith cost " << best.first 
                  << " (" << rpt.budget - best.first << " less than your budget!)"
                  << "\n\tand time " << best.second << "\n\n";
        return;
      }

      std::cout << "\n\tThere is no path from " << rpt.source << " to " << rpt.destination << " with a maximum cost of " << rpt.budget << "\n\n";

    }


    void disp_report(const ccSPBundle &rpt) {


      bool feasible = true; // stores whether or not the problem instance was valid

      if (rpt.S[rpt.destination].size() == 0)
        feasible = false;

      std::cout << "\n\tA trip from " << rpt.source 
                << " to " << rpt.destination << " is ";   

      if (!feasible)
        std::cout << "NOT ";
      std::cout << "FEASIBLE!\n";
      


      if (feasible)
        disp_paths(rpt);


      if (feasible)
        disp_fastest(rpt);
      else 
        std::cout << "\n\tThere is no path from " 
                  << rpt.source << " to " << rpt.destination 
                  << " with a maximum cost of " << rpt.budget << "\n\n";

      
        
    }

};

