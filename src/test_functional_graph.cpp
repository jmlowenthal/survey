#include <catch.hpp>
#include "functional_graph.h"
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>

BOOST_CONCEPT_ASSERT((boost::BidirectionalGraphConcept<functional_graph<int>>));