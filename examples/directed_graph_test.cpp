//=======================================================================
// Copyright 2012
// Authors: David Doria
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph

int main(int,char*[])
{
  // directed_graph is a subclass of adjacency_list which gives you object oriented access to functions
  // like add_vertex and add_edge, which makes the code easier to understand. However, it hard codes many
  // of the template parameters, so it is much less flexible.

  typedef boost::directed_graph<> Graph;
  Graph g;
  boost::graph_traits<Graph>::vertex_descriptor v0 = g.add_vertex();
  boost::graph_traits<Graph>::vertex_descriptor v1 = g.add_vertex();

  g.add_edge(v0, v1);

    // testing
    const int nedges = sizeof(used_by)/sizeof(Edge);
    int weights[nedges];
    std::fill(weights, weights + nedges, 1);

    using namespace boost;

    typedef adjacency_list< vecS, vecS, directedS,
      property< vertex_color_t, default_color_type >,
      property< edge_weight_t, int >
    > Graph;
    Graph g(used_by, used_by + nedges, weights, N);

    write_graphviz(std::cout, g, make_label_writer(name));

  return 0;
}
