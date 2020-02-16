/*
 *  Cost-constrained SSSP
 * 
 *  CS401 F19 Prog2
 *  Frank Errichiello & Tomasz Hulka
 * 
 * 
 *  Given a .txt file which contains the graph details, a source vertex, destination vertex and a budget,
 *  the program reports:
 *  - the entire tradeof curve of non-dominated paths from the specified source to the destination.
 *  - the fastest cost-feasible path (if no feasible path exists, the program reports so), the cost and 
 *    traversal time of the path as well as the path itself (as a sequence of vertices). 
 * 
 * 
 *  Usage: ./cpath <INPUT-FILE> <TARGET> <START> <DEST> <BUDGET>
 *  
 */



#include <iostream>
#include "Graph.h"


int main(int argc, char *argv[]) {
  
  graph g;
  
  if(argc != 5) {
    std::cout << "usage: cpath <file> <s> <d> <budget>\n";
    return 0;

  }

  else {
    if(!g.read_file(argv[1])) {
      std::cout << "could not open file '" << argv[1] << "'\n";
      return 0;
    }
  }

  graph::ccSPBundle rpt;


  rpt.source = std::stoi(argv[2]);
  rpt.destination = std::stoi(argv[3]);
  rpt.budget = std::stod(argv[4]);


  std::cout << "\nCost Constrained SSSP REPORT:\n";

  g.costConstrained(rpt);

  

  return 0;
}

