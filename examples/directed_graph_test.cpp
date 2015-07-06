//=======================================================================
// Copyright 2012
// Authors: David Doria
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/serialization/utility.hpp>

#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph

#include <boost/graph/graph_traits.hpp>

using namespace boost;

int main(int,char*[])
{
  // directed_graph is a subclass of adjacency_list which gives you object oriented access to functions
  // like add_vertex and add_edge, which makes the code easier to understand. However, it hard codes many
  // of the template parameters, so it is much less flexible.

  typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
  typedef boost::directed_graph<boost::no_property, EdgeWeightProperty> Graph;
  Graph g;
  boost::graph_traits<Graph>::vertex_descriptor v0 = g.add_vertex();
  boost::graph_traits<Graph>::vertex_descriptor v1 = g.add_vertex();

    EdgeWeightProperty ew = 3.1;   
  g.add_edge(v0, v1, ew);

  write_graphviz(std::cout, g, make_label_writer(get(&Edge::weight, g)));

  return 0;
}
